#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ---------- Fonts (28/18 if enabled; fallback to 14) ---------- */
extern const lv_font_t lv_font_montserrat_14;
#if defined(CONFIG_LV_FONT_MONTSERRAT_28) || defined(CONFIG_LVGL_FONT_MONTSERRAT_28)
extern const lv_font_t lv_font_montserrat_28;
#define FONT_TITLE   lv_font_montserrat_28
#define FONT_STATUS  lv_font_montserrat_28
#define FONT_BIG     lv_font_montserrat_28
#elif defined(CONFIG_LV_FONT_MONTSERRAT_18) || defined(CONFIG_LVGL_FONT_MONTSERRAT_18)
extern const lv_font_t lv_font_montserrat_18;
#define FONT_TITLE   lv_font_montserrat_18
#define FONT_STATUS  lv_font_montserrat_18
#define FONT_BIG     lv_font_montserrat_18
#else
#define FONT_TITLE   lv_font_montserrat_14
#define FONT_STATUS  lv_font_montserrat_14
#define FONT_BIG     lv_font_montserrat_14
#endif
#if defined(CONFIG_LV_FONT_MONTSERRAT_18) || defined(CONFIG_LVGL_FONT_MONTSERRAT_18)
extern const lv_font_t lv_font_montserrat_18;
#define FONT_BUTTON  lv_font_montserrat_18
#else
#define FONT_BUTTON  lv_font_montserrat_14
#endif
/* -------------------------------------------------------------- */

/* ===== Hardware bindings: UART8 on PJ8 (TX) / PJ9 (RX) ===== */
#define UART_NODE DT_NODELABEL(uart8)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

/* ===== ADC3 for Joystick (A4=PC2->CH0, A5=PC3->CH1) ===== */
#define ADC3_NODE DT_NODELABEL(adc3)
static const struct device *const adc3_dev = DEVICE_DT_GET(ADC3_NODE);

#define JOY_CH_X   0               /* ADC3_IN0 (A4/PC2) */
#define JOY_CH_Y   1               /* ADC3_IN1 (A5/PC3) */
#define JOY_RES    10              /* 10-bit to match datasheet's 0..1023 */
#define JOY_MAX    ((1 << JOY_RES) - 1)   /* 1023 */

/* Deadzone/hysteresis tuned for 10-bit */
#define DEADZONE_ENTER  90
#define DEADZONE_EXIT   60

/* ---------- UI handles (all on a single root container) ---------- */
static lv_obj_t *root_cont;
static lv_obj_t *title_lbl;
static lv_obj_t *btn_check;
static lv_obj_t *btn_change;
static lv_obj_t *status_lbl;
static lv_obj_t *btn_back;
static lv_obj_t *btn_up;
static lv_obj_t *addr_display_lbl;
static lv_obj_t *btn_down;
static lv_obj_t *btn_confirm;

/* ---------- Selection state (0=Check, 1=Change) ---------- */
static int selected_index = 0;
static const int MENU_ITEMS = 2;
static bool on_change_screen = false;

/* ---------- Scan worker thread + message queue ---------- */
struct scan_msg { char text[96]; bool done; };
K_MSGQ_DEFINE(scan_q, sizeof(struct scan_msg), 8, 4);

#define SCAN_STACK_SIZE 2048
#define SCAN_PRIO       5
K_THREAD_STACK_DEFINE(scan_stack, SCAN_STACK_SIZE);
static struct k_thread scan_thread_data;

static volatile bool scanning = false;
static volatile bool scan_cancel = false;
static volatile bool changing = false;
static bool selecting_change = false;
static int change_candidate = 1;
static int last_found_addr = -1;
static int change_old_addr = -1;
static int change_new_addr = -1;
static bool up_held = false;
static bool down_held = false;
static int64_t hold_next_ms = 0;
#define HOLD_INITIAL_DELAY_MS   350
#define HOLD_REPEAT_INTERVAL_MS 120

#define CHANGE_STACK_SIZE 2048
#define CHANGE_PRIO       5
K_THREAD_STACK_DEFINE(change_stack, CHANGE_STACK_SIZE);
static struct k_thread change_thread_data;

/* ===================== UART helpers ===================== */

/* 19200 baud, 8-O-1 (Odd parity) to match the controller */
static int uart_set_19200_8O1(const struct device *dev)
{
    struct uart_config cfg = {
        .baudrate  = 19200,
        .parity    = UART_CFG_PARITY_ODD,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    return uart_configure(dev, &cfg);
}

static void uart_flush_rx(const struct device *dev)
{
    unsigned char c;
    while (uart_poll_in(dev, &c) == 0) { /* drop */ }
}

static void uart_send_bytes(const struct device *dev, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(dev, data[i]);
    }
    /* Give the transceiver time to switch back to RX after the last byte */
    k_msleep(20);
}

/* Accumulate bytes up to total_window_ms, or end early if a silent gap >= silent_break_ms occurs */
static int uart_recv_window(uint8_t *buf, size_t cap, int total_window_ms, int silent_break_ms)
{
    int got = 0;
    int64_t deadline = k_uptime_get() + total_window_ms;
    int64_t last_rx = -1;

    while (k_uptime_get() < deadline && got < (int)cap) {
        unsigned char c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            buf[got++] = c;
            last_rx = k_uptime_get();
            continue;
        }

        /* No byte this instant */
        k_msleep(1);

        if (last_rx >= 0 && (k_uptime_get() - last_rx) >= silent_break_ms) {
            /* We already received something and then saw a quiet gap: end early */
            break;
        }
    }
    return got;
}

/* ===================== Packet template (same as Python) ===================== */
static uint8_t TEMPLATE[15] = {
    0x3F, 0x3F, 0x00, 0x37, 0xFF, 0xFB, 0x0D, 0x7E,
    0x77, 0x01, 0x00, 0x01, 0x00, 0x40, 0x75
};
#define ADDR_INDEX 3
#define CHK_INDEX  14

static uint8_t checksum_for(uint8_t addr) {
    return (addr + 0x3E) & 0xFF;
}

static size_t build_probe_frame(uint8_t addr, uint8_t *out)
{
    memcpy(out, TEMPLATE, sizeof(TEMPLATE));
    out[ADDR_INDEX] = addr;
    out[CHK_INDEX]  = checksum_for(addr);
    return sizeof(TEMPLATE);
}

/* ===================== Change-address frame ===================== */
static size_t build_change_frame(uint8_t old_addr, uint8_t new_addr, uint8_t *out)
{
    /* 00 <OLD> FF FB 10 7D 77 01 00 41 00 01 21 00 <NEW> <CHK> */
    static const uint8_t base[] = {
        0x00, 0x00, 0xFF, 0xFB, 0x10, 0x7D, 0x77, 0x01,
        0x00, 0x41, 0x00, 0x01, 0x21, 0x00, 0x00, 0x00
    };
    memcpy(out, base, sizeof(base));
    out[1]  = old_addr;
    out[14] = new_addr;
    out[15] = (uint8_t)((0x62 + old_addr + new_addr) & 0xFF); /* checksum */
    return sizeof(base);
}

/* ===================== Tuned timings (aligned to your Python) ===================== */
#define PRE_SEND_QUIET_MS     5
#define TURNAROUND_DELAY_MS   100
#define READ_WINDOW_MS        400
#define SILENT_BREAK_MS       30
#define INTER_ADDR_DELAY_MS   250
#define MIN_VALID_REPLYLEN    12

static bool probe_address(uint8_t addr)
{
    uint8_t tx[32], rx[512];
    size_t tx_len = build_probe_frame(addr, tx);

    uart_flush_rx(uart_dev);
    k_msleep(PRE_SEND_QUIET_MS);

    uart_send_bytes(uart_dev, tx, tx_len);

    k_msleep(TURNAROUND_DELAY_MS);

    int got = uart_recv_window(rx, sizeof(rx), READ_WINDOW_MS, SILENT_BREAK_MS);

    if (got >= MIN_VALID_REPLYLEN) {
        printk("Found controller at %u (RX %d bytes)\n", addr, got);
        return true;
    } else {
        if (got == 0) {
            printk("Addr %u: no reply\n", addr);
        } else {
            printk("Addr %u: short reply (%d bytes)\n", addr, got);
        }
        return false;
    }
}

static int send_change_frame(uint8_t old_addr, uint8_t new_addr, uint8_t *rx, size_t rx_cap)
{
    uint8_t tx[32];
    size_t tx_len = build_change_frame(old_addr, new_addr, tx);

    uart_flush_rx(uart_dev);
    k_msleep(PRE_SEND_QUIET_MS);

    uart_send_bytes(uart_dev, tx, tx_len);
    k_msleep(TURNAROUND_DELAY_MS);

    int got = uart_recv_window(rx, rx_cap, READ_WINDOW_MS, SILENT_BREAK_MS);

    if (got == 0) {
        printk("Change addr %u->%u: no reply\n", old_addr, new_addr);
    } else {
        printk("Change addr %u->%u: RX %d bytes\n", old_addr, new_addr, got);
    }
    return got;
}

/* ===================== Worker thread ===================== */

static void scan_thread(void *p1, void *p2, void *p3)
{
    struct scan_msg m;

    snprintf(m.text, sizeof(m.text), "Scanning...");
    m.done = false;
    k_msgq_put(&scan_q, &m, K_FOREVER);

    for (uint8_t addr = 1; addr <= 64; ++addr) {
        if (scan_cancel) {
            break;
        }

        snprintf(m.text, sizeof(m.text), "Checking addr %u...", addr);
        m.done = false;
        k_msgq_put(&scan_q, &m, K_FOREVER);

        bool present = probe_address(addr);

        if (present) {
            last_found_addr = addr;
            snprintf(m.text, sizeof(m.text), "Controller address is %u", addr);
            m.done = true;
            k_msgq_put(&scan_q, &m, K_FOREVER);
            scanning = false;
            return;
        }

        k_msleep(INTER_ADDR_DELAY_MS);
    }

    if (scan_cancel) {
        snprintf(m.text, sizeof(m.text), "Scan cancelled.");
    } else {
        snprintf(m.text, sizeof(m.text), "Scan complete. No devices.");
    }
    m.done = true;
    k_msgq_put(&scan_q, &m, K_FOREVER);
    scan_cancel = false;
    scanning = false;
}

/* ===================== Change-address worker ===================== */

static void change_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct scan_msg m;
    int old_addr = change_old_addr;
    int new_addr = change_new_addr;

    if (old_addr <= 0) {
        snprintf(m.text, sizeof(m.text), "Finding current addr...");
        m.done = false;
        k_msgq_put(&scan_q, &m, K_FOREVER);

        for (uint8_t addr = 1; addr <= 64; ++addr) {
            if (probe_address(addr)) {
                old_addr = addr;
                last_found_addr = addr;
                break;
            }
            k_msleep(INTER_ADDR_DELAY_MS);
        }

        if (old_addr <= 0) {
            snprintf(m.text, sizeof(m.text), "Change failed: no device found");
            m.done = true;
            k_msgq_put(&scan_q, &m, K_FOREVER);
            changing = false;
            return;
        }
    }

    snprintf(m.text, sizeof(m.text), "Changing %d -> %d...", old_addr, new_addr);
    m.done = false;
    k_msgq_put(&scan_q, &m, K_FOREVER);

    uint8_t rx[256];
    send_change_frame((uint8_t)old_addr, (uint8_t)new_addr, rx, sizeof(rx));

    k_msleep(150);

    bool new_ok = probe_address((uint8_t)new_addr);
    bool old_ok = probe_address((uint8_t)old_addr);

    if (new_ok && !old_ok) {
        last_found_addr = new_addr;
        snprintf(m.text, sizeof(m.text), "Success: now at %d (was %d)", new_addr, old_addr);
    } else if (new_ok && old_ok) {
        snprintf(m.text, sizeof(m.text), "Warn: responds at both %d and %d", new_addr, old_addr);
    } else if (!new_ok && old_ok) {
        snprintf(m.text, sizeof(m.text), "Failed: still at %d, not at %d", old_addr, new_addr);
    } else {
        snprintf(m.text, sizeof(m.text), "No response after change attempt");
    }
    m.done = true;
    k_msgq_put(&scan_q, &m, K_FOREVER);

    changing = false;
}

/* ===================== LVGL helpers ===================== */

static void update_highlight(void)
{
    lv_obj_set_style_border_width(btn_check, 3, 0);
    lv_obj_set_style_border_color(btn_check, lv_color_hex(0x0A4EA6), 0);

    lv_obj_set_style_border_width(btn_change, 3, 0);
    lv_obj_set_style_border_color(btn_change, lv_color_hex(0x0A4EA6), 0);

    if (selected_index == 0) {
        lv_obj_set_style_border_width(btn_check, 6, 0);
        lv_obj_set_style_border_color(btn_check, lv_color_hex(0xFFD166), 0);
    } else {
        lv_obj_set_style_border_width(btn_change, 6, 0);
        lv_obj_set_style_border_color(btn_change, lv_color_hex(0xFFD166), 0);
    }
}

/* Forward declarations */
static void start_scan(void);
static void show_change_screen(void);
static void hide_change_screen(void);
static void start_change_thread(int new_addr);
static void update_addr_display(void);
static void addr_increment(void);
static void addr_decrement(void);

/* ===================== UI callbacks ===================== */

static void btn_check_event_cb(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (on_change_screen) return;
    if (scanning) {
        scan_cancel = true;
        return;
    }
    if (changing) return;
    start_scan();
}

static void btn_change_event_cb(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (scanning || changing) return;
    if (!on_change_screen) {
        show_change_screen();
    }
}

static void update_addr_display(void)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", change_candidate);
    lv_label_set_text(addr_display_lbl, buf);
}

static void addr_increment(void)
{
    change_candidate = (change_candidate % 64) + 1;
    update_addr_display();
}

static void addr_decrement(void)
{
    change_candidate = (change_candidate - 2 + 64) % 64 + 1;
    update_addr_display();
}

static void show_change_screen(void)
{
    on_change_screen = true;
    selecting_change = true;
    change_candidate = (last_found_addr > 0) ? last_found_addr : 1;
    update_addr_display();

    /* Hide main-mode widgets */
    lv_label_set_text(title_lbl, "Change Address");
    lv_obj_add_flag(btn_check, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_change, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);

    /* Show change-mode widgets */
    lv_obj_clear_flag(btn_back, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_up, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(addr_display_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_down, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_confirm, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_pad_row(root_cont, 8, 0);
}

static void hide_change_screen(void)
{
    on_change_screen = false;
    selecting_change = false;
    up_held = false;
    down_held = false;

    /* Hide change-mode widgets */
    lv_obj_add_flag(btn_back, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_up, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(addr_display_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_down, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(btn_confirm, LV_OBJ_FLAG_HIDDEN);

    /* Show main-mode widgets */
    lv_label_set_text(title_lbl, "RS-485 Controller Tools");
    lv_obj_clear_flag(btn_check, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(btn_change, LV_OBJ_FLAG_HIDDEN);

    lv_obj_set_style_pad_row(root_cont, 24, 0);
    update_highlight();
}

/* ---------- Change-screen touch callbacks ---------- */

static void btn_up_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        up_held = true;
        addr_increment();
        hold_next_ms = k_uptime_get() + HOLD_INITIAL_DELAY_MS;
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        up_held = false;
    }
}

static void btn_down_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        down_held = true;
        addr_decrement();
        hold_next_ms = k_uptime_get() + HOLD_INITIAL_DELAY_MS;
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        down_held = false;
    }
}

static void btn_back_event_cb(lv_event_t *e)
{
    ARG_UNUSED(e);
    hide_change_screen();
}

static void btn_confirm_event_cb(lv_event_t *e)
{
    ARG_UNUSED(e);
    int addr = change_candidate;
    hide_change_screen();
    start_change_thread(addr);
}

static void start_change_thread(int new_addr)
{
    change_new_addr = new_addr;
    change_old_addr = last_found_addr;
    selecting_change = false;
    changing = true;

    lv_obj_add_state(btn_check, LV_STATE_DISABLED);
    lv_obj_add_state(btn_change, LV_STATE_DISABLED);
    lv_obj_clear_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);

    k_thread_create(&change_thread_data, change_stack, K_THREAD_STACK_SIZEOF(change_stack),
                    change_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(CHANGE_PRIO), 0, K_NO_WAIT);
}

/* Start scan (shared by button click + joystick click) */
static void start_scan(void)
{
    scanning = true;
    scan_cancel = false;

    /* Keep btn_check enabled so the user can tap it again to cancel */
    lv_obj_add_state(btn_change, LV_STATE_DISABLED);
    lv_obj_clear_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);

    k_thread_create(&scan_thread_data, scan_stack, K_THREAD_STACK_SIZEOF(scan_stack),
                    scan_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(SCAN_PRIO), 0, K_NO_WAIT);
}

/* ===================== ADC (joystick) helpers ===================== */

static int adc_setup_ch(const struct device *adc, uint8_t ch)
{
    struct adc_channel_cfg cfg = {
        .gain             = ADC_GAIN_1,
        .reference        = ADC_REF_VDD_1,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id       = ch,
    };
    return adc_channel_setup(adc, &cfg);
}

static int adc_read_ch(const struct device *adc, uint8_t ch, uint16_t *val)
{
    struct adc_sequence seq = {
        .channels    = BIT(ch),
        .buffer      = val,
        .buffer_size = sizeof(*val),
        .resolution  = JOY_RES,
    };
    return adc_read(adc, &seq);
}

/* ===================== Main ===================== */

int main(void)
{
    const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display)) {
        printk("Display not ready\n");
        return 1;
    }
    if (!device_is_ready(uart_dev)) {
        printk("UART8 not ready\n");
        return 1;
    }
    if (uart_set_19200_8O1(uart_dev)) {
        printk("UART8 configure failed\n");
        return 1;
    }

    /* ---- Joystick ADC init ---- */
    if (!device_is_ready(adc3_dev)) {
        printk("ADC3 not ready\n");
        return 1;
    }
    adc_setup_ch(adc3_dev, JOY_CH_X);
    adc_setup_ch(adc3_dev, JOY_CH_Y);

    /* ---- UI: single root container, toggle widget visibility for modes ---- */
    root_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(root_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root_cont, 0, 0);
    lv_obj_set_flex_flow(root_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(root_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(root_cont, 24, 0);

    /* Title (shared, text changes between modes) */
    title_lbl = lv_label_create(root_cont);
    lv_label_set_text(title_lbl, "RS-485 Controller Tools");
    lv_obj_set_width(title_lbl, LV_PCT(100));
    lv_obj_set_style_text_align(title_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(title_lbl, &FONT_TITLE, LV_PART_MAIN | LV_STATE_DEFAULT);

    /*
     * Widget creation order determines flex layout order.
     * Hidden widgets are skipped by flex, so both modes lay out correctly.
     *
     * Change-mode: title, btn_back, btn_up, addr_display, btn_down, btn_confirm
     * Main-mode:   title, btn_check, btn_change, status_lbl
     */

    /* -- Change-mode widgets (hidden initially) -- */

    btn_back = lv_btn_create(root_cont);
    lv_obj_set_size(btn_back, LV_PCT(50), 38);
    lv_obj_set_style_radius(btn_back, 12, 0);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_color(btn_back, lv_color_hex(0x666666), 0);
    lv_obj_set_style_border_width(btn_back, 2, 0);
    lv_obj_add_event_cb(btn_back, btn_back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_margin_bottom(btn_back, 12, 0);
    lv_obj_add_flag(btn_back, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_font(lbl_back, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_back);

    btn_up = lv_btn_create(root_cont);
    lv_obj_set_size(btn_up, LV_PCT(65), 55);
    lv_obj_set_style_radius(btn_up, 14, 0);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0x2D6A4F), 0);
    lv_obj_set_style_border_color(btn_up, lv_color_hex(0x1B4332), 0);
    lv_obj_set_style_border_width(btn_up, 3, 0);
    lv_obj_add_event_cb(btn_up, btn_up_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(btn_up, btn_up_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(btn_up, btn_up_event_cb, LV_EVENT_PRESS_LOST, NULL);
    lv_obj_add_flag(btn_up, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *lbl_up = lv_label_create(btn_up);
    lv_label_set_text(lbl_up, LV_SYMBOL_UP);
    lv_obj_set_style_text_font(lbl_up, &FONT_BIG, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_up);

    addr_display_lbl = lv_label_create(root_cont);
    lv_label_set_text(addr_display_lbl, "1");
    lv_obj_set_style_text_font(addr_display_lbl, &FONT_BIG, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(addr_display_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(addr_display_lbl, LV_OBJ_FLAG_HIDDEN);

    btn_down = lv_btn_create(root_cont);
    lv_obj_set_size(btn_down, LV_PCT(65), 55);
    lv_obj_set_style_radius(btn_down, 14, 0);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0x2D6A4F), 0);
    lv_obj_set_style_border_color(btn_down, lv_color_hex(0x1B4332), 0);
    lv_obj_set_style_border_width(btn_down, 3, 0);
    lv_obj_add_event_cb(btn_down, btn_down_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(btn_down, btn_down_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(btn_down, btn_down_event_cb, LV_EVENT_PRESS_LOST, NULL);
    lv_obj_add_flag(btn_down, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *lbl_down = lv_label_create(btn_down);
    lv_label_set_text(lbl_down, LV_SYMBOL_DOWN);
    lv_obj_set_style_text_font(lbl_down, &FONT_BIG, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_down);

    /* -- Main-mode widgets -- */

    btn_check = lv_btn_create(root_cont);
    lv_obj_set_size(btn_check, LV_PCT(65), 60);
    lv_obj_set_style_radius(btn_check, 18, 0);
    lv_obj_set_style_bg_color(btn_check, lv_color_hex(0x0F66D0), 0);
    lv_obj_set_style_border_color(btn_check, lv_color_hex(0x0A4EA6), 0);
    lv_obj_set_style_border_width(btn_check, 3, 0);
    lv_obj_add_event_cb(btn_check, btn_check_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_check = lv_label_create(btn_check);
    lv_label_set_text(lbl_check, "Check Address");
    lv_obj_set_style_text_font(lbl_check, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_check);

    btn_change = lv_btn_create(root_cont);
    lv_obj_set_size(btn_change, LV_PCT(65), 60);
    lv_obj_set_style_radius(btn_change, 18, 0);
    lv_obj_set_style_bg_color(btn_change, lv_color_hex(0x155E63), 0);
    lv_obj_set_style_border_color(btn_change, lv_color_hex(0x0A4EA6), 0);
    lv_obj_set_style_border_width(btn_change, 3, 0);
    lv_obj_add_event_cb(btn_change, btn_change_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_change = lv_label_create(btn_change);
    lv_label_set_text(lbl_change, "Change Address");
    lv_obj_set_style_text_font(lbl_change, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_change);

    /* -- Change-mode confirm (hidden initially, placed after main btns in creation order) -- */

    btn_confirm = lv_btn_create(root_cont);
    lv_obj_set_size(btn_confirm, LV_PCT(65), 45);
    lv_obj_set_style_radius(btn_confirm, 14, 0);
    lv_obj_set_style_bg_color(btn_confirm, lv_color_hex(0x0F66D0), 0);
    lv_obj_set_style_border_color(btn_confirm, lv_color_hex(0x0A4EA6), 0);
    lv_obj_set_style_border_width(btn_confirm, 3, 0);
    lv_obj_add_event_cb(btn_confirm, btn_confirm_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_margin_top(btn_confirm, 12, 0);
    lv_obj_add_flag(btn_confirm, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *lbl_confirm = lv_label_create(btn_confirm);
    lv_label_set_text(lbl_confirm, "Confirm");
    lv_obj_set_style_text_font(lbl_confirm, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_confirm);

    /* Status label (main-mode, hidden until a scan/change starts) */
    status_lbl = lv_label_create(root_cont);
    lv_obj_add_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(status_lbl, LV_PCT(100));
    lv_obj_set_style_text_align(status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(status_lbl, &FONT_STATUS, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_long_mode(status_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);

    display_blanking_off(display);
    lv_timer_handler();

    /* Calibrate joystick center */
    int32_t cx = 0, cy = 0;
    for (int i = 0; i < 64; i++) {
        uint16_t vx=0, vy=0;
        adc_read_ch(adc3_dev, JOY_CH_X, &vx);
        adc_read_ch(adc3_dev, JOY_CH_Y, &vy);
        cx += vx; cy += vy;
        k_msleep(5);
    }
    int center_x = cx / 64;
    int center_y = cy / 64;
    printk("Center calibration: X=%d  Y=%d\n", center_x, center_y);

    int last_dir_x = 0, last_dir_y = 0, last_click = 0;
    int64_t last_press_ms = -1000000;
    #define PRESS_DEBOUNCE_MS 200
    update_highlight();

    /* ---- Main loop ---- */
    while (1) {
        /* Pump scan thread messages into LVGL */
        struct scan_msg m;
        while (k_msgq_get(&scan_q, &m, K_NO_WAIT) == 0) {
            lv_label_set_text(status_lbl, m.text);
            lv_obj_clear_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);
            if (m.done) {
                lv_obj_clear_state(btn_check, LV_STATE_DISABLED);
                lv_obj_clear_state(btn_change, LV_STATE_DISABLED);
            }
        }

        /* Read joystick */
        uint16_t x_raw=0, y_raw=0;
        adc_read_ch(adc3_dev, JOY_CH_X, &x_raw);
        adc_read_ch(adc3_dev, JOY_CH_Y, &y_raw);

        int dx = (int)x_raw - center_x;
        int dy = (int)y_raw - center_y;

        int64_t now_ms = k_uptime_get();

        /*
         * "Press" detection on 4-pin joystick: X forced to max (1023).
         * Treat press as a rising-edge click and ignore UP/DOWN movement for a short
         * debounce window. This prevents a physical press from also being read as
         * an unintended UP/DOWN step (menu select or address increment).
         */
        int pressed = (x_raw >= JOY_MAX);
        bool click_rising = (pressed && !last_click);
        if (click_rising) {
            last_press_ms = now_ms;

            if (on_change_screen) {
                btn_confirm_event_cb(NULL);
            } else if (!scanning && !changing) {
                if (selected_index == 0) {
                    btn_check_event_cb(NULL);
                } else {
                    btn_change_event_cb(NULL);
                }
            } else if (scanning) {
                scan_cancel = true;
            }
        }
        last_click = pressed;

        /* During debounce window, ignore direction changes */
        if ((now_ms - last_press_ms) < PRESS_DEBOUNCE_MS) {
            lv_timer_handler();
            k_msleep(10);
            continue;
        }

        int dir_x = last_dir_x;
        int dir_y = last_dir_y;

        /* X axis: LEFT triggers "Back" on change screen */
        if (last_dir_x == 0) {
            if (dx < -DEADZONE_ENTER) dir_x = -1;
            else if (dx > DEADZONE_ENTER) dir_x = +1;
        } else if (last_dir_x == -1 && dx > -DEADZONE_EXIT) {
            dir_x = 0;
        } else if (last_dir_x == +1 && dx < DEADZONE_EXIT) {
            dir_x = 0;
        }

        if (dir_x != last_dir_x && dir_x == -1 && on_change_screen) {
            hide_change_screen();
        }
        last_dir_x = dir_x;

        /* Y axis */
        if (last_dir_y == 0) {
            if (dy < -DEADZONE_ENTER) dir_y = -1;   /* UP */
            else if (dy > DEADZONE_ENTER) dir_y = +1; /* DOWN */
        } else if (last_dir_y == -1 && dy > -DEADZONE_EXIT) {
            dir_y = 0;
        } else if (last_dir_y == +1 && dy < DEADZONE_EXIT) {
            dir_y = 0;
        }

        if (dir_y != last_dir_y) {
            if (on_change_screen) {
                if (dir_y == -1) {
                    addr_increment();
                } else if (dir_y == +1) {
                    addr_decrement();
                }
                last_dir_y = dir_y;
            } else {
                if (dir_y == -1) {
                    selected_index = (selected_index - 1 + MENU_ITEMS) % MENU_ITEMS;
                    update_highlight();
                } else if (dir_y == +1) {
                    selected_index = (selected_index + 1) % MENU_ITEMS;
                    update_highlight();
                }
                last_dir_y = dir_y;
            }
        }

        /* Hold-to-repeat for touch UP/DOWN buttons */
        if (up_held && k_uptime_get() >= hold_next_ms) {
            addr_increment();
            hold_next_ms = k_uptime_get() + HOLD_REPEAT_INTERVAL_MS;
        }
        if (down_held && k_uptime_get() >= hold_next_ms) {
            addr_decrement();
            hold_next_ms = k_uptime_get() + HOLD_REPEAT_INTERVAL_MS;
        }

        lv_timer_handler();
        k_msleep(10);
    }
}
