#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/uart.h>
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

/* ---------- UI handles ---------- */
static lv_obj_t *status_lbl;
static lv_obj_t *btn;

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
/* Increase these compared to your previous C attempt */
#define PRE_SEND_QUIET_MS     5     /* small quiet time before TX */
#define TURNAROUND_DELAY_MS   100   /* TX->RX gap; Gravity auto-direction needs time */
#define READ_WINDOW_MS        400   /* allow long replies or slow turnaround */
#define SILENT_BREAK_MS       30    /* end early after quiet gap once data starts */
#define INTER_ADDR_DELAY_MS   250   /* pacing between probes */
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
        printk("✅ Found controller at %u (RX %d bytes)\n", addr, got);
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
            snprintf(m.text, sizeof(m.text), "✅ Found device at %u", addr);
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

/* ===================== UI callbacks ===================== */

static void btn_event_cb(lv_event_t *e)
{
    if (scanning) return;
    scanning = true;

    lv_obj_add_state(btn, LV_STATE_DISABLED);
    lv_obj_clear_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);

    k_thread_create(&scan_thread_data, scan_stack, K_THREAD_STACK_SIZEOF(scan_stack),
                    scan_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(SCAN_PRIO), 0, K_NO_WAIT);
}

/* ===================== Main ===================== */

void main(void)
{
    const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display)) {
        printk("Display not ready\n");
        return;
    }
    if (!device_is_ready(uart_dev)) {
        printk("UART8 not ready\n");
        return;
    }

    if (uart_set_19200_8O1(uart_dev)) {
        printk("UART8 configure failed\n");
        return;
    }

    lv_obj_t *root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(root, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(root, 24, 0);

    lv_obj_t *title = lv_label_create(root);
    lv_label_set_text(title, "RS-485 Controller Scanner");
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(title, &FONT_TITLE, LV_PART_MAIN | LV_STATE_DEFAULT);

    btn = lv_btn_create(root);
    lv_obj_set_size(btn, LV_PCT(90), 96);
    lv_obj_set_style_radius(btn, 18, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0F66D0), 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x0A4EA6), 0);
    lv_obj_set_style_border_width(btn, 3, 0);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_lbl = lv_label_create(btn);
    lv_label_set_text(btn_lbl, "Scan Controllers");
    lv_obj_set_style_text_font(btn_lbl, &FONT_BUTTON, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(btn_lbl);

    status_lbl = lv_label_create(root);
    lv_obj_add_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(status_lbl, LV_PCT(100));
    lv_obj_set_style_text_align(status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(status_lbl, &FONT_STATUS, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_long_mode(status_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);

    display_blanking_off(display);
    lv_timer_handler();

    while (1) {
        struct scan_msg m;
        while (k_msgq_get(&scan_q, &m, K_NO_WAIT) == 0) {
            lv_label_set_text(status_lbl, m.text);
            if (m.done) {
                lv_obj_clear_state(btn, LV_STATE_DISABLED);
            }
        }
        lv_timer_handler();
        k_msleep(10);
    }
}
