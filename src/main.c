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
#elif defined(CONFIG_LV_FONT_MONTSERRAT_18) || defined(CONFIG_LVGL_FONT_MONTSERRAT_18)
extern const lv_font_t lv_font_montserrat_18;
#define FONT_TITLE   lv_font_montserrat_18
#define FONT_STATUS  lv_font_montserrat_18
#else
#define FONT_TITLE   lv_font_montserrat_14
#define FONT_STATUS  lv_font_montserrat_14
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
#define JOY_RES    10              /* 10-bit to match datasheet’s 0..1023 */
#define JOY_MAX    ((1 << JOY_RES) - 1)   /* 1023 */

/* Deadzone/hysteresis tuned for 10-bit */
#define DEADZONE_ENTER  90
#define DEADZONE_EXIT   60

/* ---------- UI handles ---------- */
static lv_obj_t *status_lbl;
static lv_obj_t *btn_check;   /* “Check Address” */
static lv_obj_t *btn_change;  /* “Change Address” */

/* ---------- Selection state (0=Check, 1=Change) ---------- */
static int selected_index = 0;
static const int MENU_ITEMS = 2;

/* ---------- Scan worker thread + message queue ---------- */
struct scan_msg { char text[96]; bool done; };
K_MSGQ_DEFINE(scan_q, sizeof(struct scan_msg), 8, 4);

#define SCAN_STACK_SIZE 2048
#define SCAN_PRIO       5
K_THREAD_STACK_DEFINE(scan_stack, SCAN_STACK_SIZE);
static struct k_thread scan_thread_data;

static volatile bool scanning = false;

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

    /* Ensure clean RX FIFO and a brief idle period before sending */
    uart_flush_rx(uart_dev);
    k_msleep(PRE_SEND_QUIET_MS);

    /* Send probe (Gravity RS-485 does auto TX/RX switching) */
    uart_send_bytes(uart_dev, tx, tx_len);

    /* TX->RX turnaround */
    k_msleep(TURNAROUND_DELAY_MS);

    /* Accumulate full reply with early-quiet break */
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

/* ===================== Worker thread ===================== */

static void scan_thread(void *p1, void *p2, void *p3)
{
    struct scan_msg m;

    snprintf(m.text, sizeof(m.text), "Scanning...");
    m.done = false;
    k_msgq_put(&scan_q, &m, K_FOREVER);

    for (uint8_t addr = 1; addr <= 64; ++addr) {
        snprintf(m.text, sizeof(m.text), "Checking addr %u...", addr);
        m.done = false;
        k_msgq_put(&scan_q, &m, K_FOREVER);

        bool present = probe_address(addr);

        if (present) {
            snprintf(m.text, sizeof(m.text), "Controller address is %u", addr);
            m.done = true;
            k_msgq_put(&scan_q, &m, K_FOREVER);
            scanning = false;
            return;   // stop after first hit
        }

        k_msleep(INTER_ADDR_DELAY_MS);
    }

    snprintf(m.text, sizeof(m.text), "Scan complete. No devices.");
    m.done = true;
    k_msgq_put(&scan_q, &m, K_FOREVER);
    scanning = false;
}

/* ===================== LVGL helpers ===================== */

static void update_highlight(void)
{
    /* Common base style */
    lv_obj_set_style_border_width(btn_check, 3, 0);
    lv_obj_set_style_border_color(btn_check, lv_color_hex(0x0A4EA6), 0);

    lv_obj_set_style_border_width(btn_change, 3, 0);
    lv_obj_set_style_border_color(btn_change, lv_color_hex(0x0A4EA6), 0);

    /* Focused one gets thicker yellow border */
    if (selected_index == 0) {
        lv_obj_set_style_border_width(btn_check, 6, 0);
        lv_obj_set_style_border_color(btn_check, lv_color_hex(0xFFD166), 0);
    } else {
        lv_obj_set_style_border_width(btn_change, 6, 0);
        lv_obj_set_style_border_color(btn_change, lv_color_hex(0xFFD166), 0);
    }
}

static void set_status(const char *txt)
{
    lv_obj_clear_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(status_lbl, txt);
}

/* Forward decl so we can call from joystick "click" */
static void start_scan(void);

/* ===================== UI callbacks ===================== */

static void btn_check_event_cb(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (scanning) return;
    start_scan();
}

static void btn_change_event_cb(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (scanning) return;

    /* Placeholder “Change Address” action */
    set_status("Change Address selected (TODO)");
    printk(">>> [ACTION] Change Address triggered (placeholder)\n");
}

/* Start scan (shared by button click + joystick click) */
static void start_scan(void)
{
    scanning = true;

    lv_obj_add_state(btn_check, LV_STATE_DISABLED);
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



    /* ---- UI ---- */
    lv_obj_t *root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(root, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(root, 24, 0);

    lv_obj_t *title = lv_label_create(root);
    lv_label_set_text(title, "RS-485 Controller Tools");
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(title, &FONT_TITLE, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Button: Check Address (scan) */
    btn_check = lv_btn_create(root);
    lv_obj_set_size(btn_check, LV_PCT(90), 80);
    lv_obj_set_style_radius(btn_check, 18, 0);
    lv_obj_set_style_bg_color(btn_check, lv_color_hex(0x0F66D0), 0);
    lv_obj_set_style_border_color(btn_check, lv_color_hex(0x0A4EA6), 0);
    lv_obj_set_style_border_width(btn_check, 3, 0);
    lv_obj_add_event_cb(btn_check, btn_check_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_check = lv_label_create(btn_check);
    lv_label_set_text(lbl_check, "Check Address");
    lv_obj_set_style_text_font(lbl_check, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_check);

    /* Button: Change Address */
    btn_change = lv_btn_create(root);
    lv_obj_set_size(btn_change, LV_PCT(90), 80);
    lv_obj_set_style_radius(btn_change, 18, 0);
    lv_obj_set_style_bg_color(btn_change, lv_color_hex(0x155E63), 0);
    lv_obj_set_style_border_color(btn_change, lv_color_hex(0x0A4EA6), 0);
    lv_obj_set_style_border_width(btn_change, 3, 0);
    lv_obj_add_event_cb(btn_change, btn_change_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_change = lv_label_create(btn_change);
    lv_label_set_text(lbl_change, "Change Address");
    lv_obj_set_style_text_font(lbl_change, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(lbl_change);

    /* Status label */
    status_lbl = lv_label_create(root);
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

        int dir_x = last_dir_x;
        int dir_y = last_dir_y;

        /* X axis reserved (future menus) */
        if (last_dir_x == 0) {
            if (dx < -DEADZONE_ENTER) dir_x = -1;
            else if (dx > DEADZONE_ENTER) dir_x = +1;
        } else if (last_dir_x == -1 && dx > -DEADZONE_EXIT) {
            dir_x = 0;
        } else if (last_dir_x == +1 && dx < DEADZONE_EXIT) {
            dir_x = 0;
        }
        last_dir_x = dir_x;

        /* Y axis controls selection */
        if (last_dir_y == 0) {
            if (dy < -DEADZONE_ENTER) dir_y = -1;   /* UP */
            else if (dy > DEADZONE_ENTER) dir_y = +1; /* DOWN */
        } else if (last_dir_y == -1 && dy > -DEADZONE_EXIT) {
            dir_y = 0;
        } else if (last_dir_y == +1 && dy < DEADZONE_EXIT) {
            dir_y = 0;
        }

        if (dir_y != last_dir_y) {
            if (dir_y == -1) {
                selected_index = (selected_index - 1 + MENU_ITEMS) % MENU_ITEMS;
                update_highlight();
            } else if (dir_y == +1) {
                selected_index = (selected_index + 1) % MENU_ITEMS;
                update_highlight();
            }
            last_dir_y = dir_y;
        }

        /* “Press” detection on 4-pin joystick: X forced to max (1023) */
        int pressed = (x_raw >= JOY_MAX);
        if (pressed != last_click) {
            if (pressed) {
                /* Trigger selected item */
                if (!scanning) {
                    if (selected_index == 0) {
                        /* Check Address behaves exactly like before */
                        btn_check_event_cb(NULL);
                    } else {
                        btn_change_event_cb(NULL);
                    }
                }
            }
            last_click = pressed;
        }

        lv_timer_handler();
        k_msleep(10);
    }
}
