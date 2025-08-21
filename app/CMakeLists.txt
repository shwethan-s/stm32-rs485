cmake_minimum_required(VERSION 3.20.0)

# Tell CMake to load Zephyr's build system from your local Zephyr install
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


project(stm32_rs485)

# List all source files here 
target_sources(app PRIVATE src/main.c)
