#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void)
{
    printk("Hello from RS485 project on STM32H747!\n");

    while (1) {
        k_sleep(K_SECONDS(1));
        printk("Tick...\n");
    }
}
