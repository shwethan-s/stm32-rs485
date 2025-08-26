cmake_minimum_required(VERSION 3.20.0)

# Tell Zephyr this is an application
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32_rs485)

# Your sources
target_sources(app PRIVATE
    src/main.c
)

# If you later add BSP drivers (like NT35510), list them here:
# target_sources(app PRIVATE
#     app/drivers/lcd/stm32h747i_discovery_lcd.c
#     app/drivers/lcd/nt35510.c
# )
