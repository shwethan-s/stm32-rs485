#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <stdio.h>

/* ---------- Fonts (28/18 if enabled; fallback to 14) ---------- */
extern const lv_font_t lv_font_montserrat_14;

#if defined(CONFIG_LV_FONT_MONTSERRAT_28)
extern const lv_font_t lv_font_montserrat_28;
#define FONT_TITLE   lv_font_montserrat_28
#define FONT_STATUS  lv_font_montserrat_28
#elif defined(CONFIG_LV_FONT_MONTSERRAT_18)
extern const lv_font_t lv_font_montserrat_18;
#define FONT_TITLE   lv_font_montserrat_18
#define FONT_STATUS  lv_font_montserrat_18
#else
#define FONT_TITLE   lv_font_montserrat_14
#define FONT_STATUS  lv_font_montserrat_14
#endif

#if defined(CONFIG_LV_FONT_MONTSERRAT_18)
extern const lv_font_t lv_font_montserrat_18;
#define FONT_BUTTON  lv_font_montserrat_18
#else
#define FONT_BUTTON  lv_font_montserrat_14
#endif
/* -------------------------------------------------------------- */

/* UI handles */
static lv_obj_t *status_lbl;
static lv_obj_t *btn;

/* Scan worker thread + message queue */
struct scan_msg { char text[64]; bool done; };
K_MSGQ_DEFINE(scan_q, sizeof(struct scan_msg), 8, 4);

#define SCAN_STACK_SIZE 2048
#define SCAN_PRIO       5
K_THREAD_STACK_DEFINE(scan_stack, SCAN_STACK_SIZE);
static struct k_thread scan_thread_data;

static volatile bool scanning = false;

/* Stub for your real RS-485 test later */
static bool is_device_present(int addr)
{
    ARG_UNUSED(addr);
    return false; /* replace with real poll+response soon */
}

/* Worker: runs off the UI thread, pushes progress to msgq */
static void scan_thread(void *p1, void *p2, void *p3)
{
    struct scan_msg m;

    snprintf(m.text, sizeof(m.text), "Scanning...");
    m.done = false;
    k_msgq_put(&scan_q, &m, K_FOREVER);

    for (int addr = 1; addr <= 64; addr++) {
        snprintf(m.text, sizeof(m.text), "Checking addr %d...", addr);
        m.done = false;
        k_msgq_put(&scan_q, &m, K_FOREVER);

        /* simulate bus time; tune as you like (fast UI feedback) */
        k_msleep(60);

        if (is_device_present(addr)) {
            /* You could buffer hits and print them at the end if you want */
        }
    }

    snprintf(m.text, sizeof(m.text), "Scan complete: no controllers found");
    m.done = true;
    k_msgq_put(&scan_q, &m, K_FOREVER);

    scanning = false;
}

/* Button click: start worker if not already running */
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

void main(void)
{
    const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display)) {
        printk("Display not ready\n");
        return;
    }

    /* Root layout */
    lv_obj_t *root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(root, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(root, 24, 0);

    /* Title */
    lv_obj_t *title = lv_label_create(root);
    lv_label_set_text(title, "RS-485 Controller Scanner");
    lv_obj_set_width(title, LV_PCT(100));
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(title, &FONT_TITLE, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* Button */
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

    /* Status label: hidden until scan starts */
    status_lbl = lv_label_create(root);
    lv_obj_add_flag(status_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_width(status_lbl, LV_PCT(100));
    lv_obj_set_style_text_align(status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(status_lbl, &FONT_STATUS, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_long_mode(status_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);

    display_blanking_off(display);
    lv_timer_handler();

    /* UI thread: drain msgq → update LVGL → keep UI responsive */
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
