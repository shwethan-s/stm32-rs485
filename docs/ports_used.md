# Ports / Pins Used (STM32-RS485)

This page lists the STM32 pins/peripherals used by this firmware, separated by feature (RS‑485 vs joystick).

## RS‑485 (Controller Bus UART)
This is the UART used to talk to the RS‑485 controller/device.

| What it is | Board label (digital) | Peripheral signal | STM32 pin | Notes |
|---|---|---|---|---|
| RS‑485 transmit (STM32 sends bytes out onto the bus) | D1 | UART8_TX | PJ8 | Configured as UART8 TX (AF8) |
| RS‑485 receive (STM32 listens for bytes from the bus) | D0 | UART8_RX | PJ9 | Configured as UART8 RX (AF8) |

- Firmware uses `DT_NODELABEL(uart8)` for the RS‑485 UART device.
- UART settings in firmware: **19200 baud, 8 data bits, odd parity, 1 stop bit (8O1)**.

## Joystick (Analog Inputs)
This is the joystick read as two analog voltages (X and Y) using the ADC.

| What it is | Board label (analog) | Peripheral | STM32 pin | Used in code |
|---|---|---|---|---|
| Joystick X axis voltage | A4 | ADC3 | PC2 | `JOY_CH_X = 0` (ADC3 channel 0) |
| Joystick Y axis voltage | A5 | ADC3 | PC3 | `JOY_CH_Y = 1` (ADC3 channel 1) |

- Firmware uses `DT_NODELABEL(adc3)` for joystick reads.
- The **A4/A5 labels** come from the comment in `src/main.c` (`A4=PC2`, `A5=PC3`).

## Debug console (logs) vs RS‑485 (important distinction)
These are not the same UART.

| What it is | Peripheral | Where it goes | Notes |
|---|---|---|---|
| Console logs (`printk`) | USART1 | ST‑LINK VCP (virtual COM port) | Overlay sets `zephyr,console = &usart1` |
| RS‑485 data | UART8 | D0/D1 (PJ9/PJ8) to RS‑485 transceiver | Used for probing/changing address |

- Note: the RS‑485 pins are shown as **D0/D1** for the board header labels, and **PJ9/PJ8** for the underlying STM32 pin names used in code/overlay.

## Display controller

| What it is | Peripheral | Notes |
|---|---|---|
| LCD display controller | LTDC | Enabled in overlay (`&ltdc { status = "okay"; };`) |


