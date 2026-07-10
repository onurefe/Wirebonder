cmake_minimum_required(VERSION 3.20)
project(firmware C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(tests)
    return()
endif()

# MCU flags
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

set(COMMON_COMPILE_OPTIONS
    ${MCU_FLAGS}
    -Og
    -Wall
    -fdata-sections
    -ffunction-sections
    -g
    -gdwarf-2
    -MMD
    -MP
)

# CMSIS-DSP fast math sources
set(CMSIS_DSP_SOURCES
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
)

# C sources (HAL + core)
set(C_SOURCES
    Core/Src/main.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/system_stm32f4xx.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# C++ application sources
set(CPP_SOURCES
    Core/Src/adc_service.cpp
    Core/Src/bonder_module.cpp
    Core/Src/transducer_analyzer.cpp
    Core/Src/dac_service.cpp
    Core/Src/fast_io.cpp
    Core/Src/force_coil_module.cpp
    Core/Src/lvdt_module.cpp
    Core/Src/leaky_integrator_controller.cpp
    Core/Src/pid_controller.cpp
    Core/Src/pin_monitor_service.cpp
    Core/Src/pll_module.cpp
    Core/Src/pwm_service.cpp
    Core/Src/robot.cpp
    Core/Src/stepper_router_service.cpp
    Core/Src/solenoid_service.cpp
    Core/Src/stepper_service.cpp
    Core/Src/timer_expire_service.cpp
    Core/Src/us_impedance_scanner_module.cpp
    Core/Src/dc_motor_velocity_controller_module.cpp
    Core/Src/dc_motor_position_controller_module.cpp
    Core/Src/io_expander_service.cpp
    Core/Src/lcd_module.cpp
    Core/Src/control_panel_service.cpp
    Core/Src/ui_module.cpp
    Core/Src/eeprom_emulator.cpp
    Core/Src/debug_service.cpp
    Core/Src/debug_keypad.cpp
    Core/Src/debug_pll.cpp
    Core/Src/debug_tone_generator.cpp
    Core/Src/debug_impedance_scanner.cpp
    Core/Src/debug_motor_velocity_controller.cpp
    Core/Src/debug_force_coil.cpp
    Core/Src/debug_motor_position_controller.cpp
    Core/Src/debug_stepper_router.cpp
)

set(ASM_SOURCES
    startup_stm32f446xx.s
)

add_executable(firmware
    ${C_SOURCES}
    ${CMSIS_DSP_SOURCES}
    ${CPP_SOURCES}
    ${ASM_SOURCES}
)

target_include_directories(firmware PRIVATE
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
)

target_compile_definitions(firmware PRIVATE
    USE_HAL_DRIVER
    STM32F446xx
    ARM_MATH_CM4
)

target_compile_options(firmware PRIVATE
    ${COMMON_COMPILE_OPTIONS}
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>
)

target_link_options(firmware PRIVATE
    ${MCU_FLAGS}
    -specs=nano.specs
    -T${CMAKE_SOURCE_DIR}/stm32f446retx_flash.ld
    -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map,--cref
    -Wl,--gc-sections
    -lc
    -lm
    -lnosys
)

# Post-build: generate .hex and .bin, print size
add_custom_command(TARGET firmware POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:firmware> ${CMAKE_BINARY_DIR}/firmware.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:firmware> ${CMAKE_BINARY_DIR}/firmware.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:firmware>
    COMMENT "Generating firmware.hex / firmware.bin"
)
