#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

static lv_obj_t *label;

static void btn_event_cb(lv_event_t *e)
{
    lv_label_set_text(label, "🔎 Scanning...");
    k_sleep(K_SECONDS(2));
    lv_label_set_text(label, "✅ Controller found at addr 19");
}

void main(void)
{
    const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        printk("Display not ready\n");
        return;
    }

    /* Button */
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Scan Controllers");

    /* Status label */
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Idle");
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -20);

    display_blanking_off(display_dev);

    while (1) {
        lv_task_handler();
        k_sleep(K_MSEC(10));
    }
}
