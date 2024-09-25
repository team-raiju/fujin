# =================================
# ==== Set arm toolchain variables
# =================================
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g0")
endif()

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS} -u _printf_float")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/cube/STM32G474RCTx_FLASH.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

# =================================
# ==== Configure target
# =================================
set(ST_ROOT $ENV{ST_ROOT})
if(NOT ST_ROOT)
    set(ST_ROOT "/opt/st")
endif()

message("ST_ROOT: " ${ST_ROOT})

set(DRIVERS_PATH ${ST_ROOT}/STM32CubeG4/Drivers)
set(MEMS_DRIVERS_PATH ${ST_ROOT}/X-CUBE-MEMS1/Drivers)
set(MIDDLEWARES_PATH ${ST_ROOT}/STM32CubeG4/Middlewares/ST)
set(MEMS_MIDDLEWARES_PATH ${ST_ROOT}/X-CUBE-MEMS1/Middlewares/ST)

if (NOT EXISTS ${DRIVERS_PATH})
    message(FATAL_ERROR "Could not find STM32CubeG4 folder in ST_ROOT")
endif()

if (NOT EXISTS ${MEMS_DRIVERS_PATH})
    message(FATAL_ERROR "Could not find X-CUBE-MEMS1 folder in ST_ROOT")
endif()

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32G474xx
)

set(BSP_PATH src/bsp/target-stm32g474)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # ST's HAL needs stm32g4xx_hal_conf to be available globally, so we need to add this here
    ${BSP_PATH}/st/inc

    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Inc
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Inc/Legacy
    ${DRIVERS_PATH}/CMSIS/Device/ST/STM32G4xx/Include
    ${DRIVERS_PATH}/CMSIS/Include
    ${MIDDLEWARES_PATH}/STM32_USB_Device_Library/Core/Inc
    ${MIDDLEWARES_PATH}/STM32_USB_Device_Library/Class/CDC/Inc
    ${MEMS_DRIVERS_PATH}/BSP/Components/lsm6dsr
    ${MEMS_MIDDLEWARES_PATH}/STM32_MotionGC_Library/Inc
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${BSP_PATH}/devices/AS5047P.cpp
    # ${BSP_PATH}/devices/eeprom_24lc512.c

    ${BSP_PATH}/st/src/config.c
    ${BSP_PATH}/st/src/stm32g4xx_hal_msp.c
    ${BSP_PATH}/st/src/stm32g4xx_it.c
    ${BSP_PATH}/st/src/syscalls.c
    ${BSP_PATH}/st/src/sysmem.c
    ${BSP_PATH}/st/src/usb_device.c
    ${BSP_PATH}/st/src/usbd_cdc_if.c
    ${BSP_PATH}/st/src/usbd_conf.c
    ${BSP_PATH}/st/src/usbd_desc.c


    ${BSP_PATH}/core.cpp
    ${BSP_PATH}/analog_sensors.cpp
    ${BSP_PATH}/ble.cpp
    ${BSP_PATH}/buttons.cpp
    ${BSP_PATH}/buzzer.cpp
    ${BSP_PATH}/eeprom.cpp
    ${BSP_PATH}/encoders.cpp
    ${BSP_PATH}/fan.cpp
    ${BSP_PATH}/imu.cpp
    ${BSP_PATH}/leds.cpp
    ${BSP_PATH}/motors.cpp
    ${BSP_PATH}/timers.cpp
    ${BSP_PATH}/usb.cpp

    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    ${DRIVERS_PATH}/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c

    ${MIDDLEWARES_PATH}/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${MIDDLEWARES_PATH}/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${MIDDLEWARES_PATH}/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${MIDDLEWARES_PATH}/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

    ${DRIVERS_PATH}/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c
    ${DRIVERS_PATH}/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g474xx.s

    ${MEMS_DRIVERS_PATH}/BSP/Components/lsm6dsr/lsm6dsr.c
    ${MEMS_DRIVERS_PATH}/BSP/Components/lsm6dsr/lsm6dsr_reg.c
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    ${MEMS_MIDDLEWARES_PATH}/STM32_MotionGC_Library/Lib/MotionGC_CM4F_wc32_ot_hard.a
)

# Execute post-build to print size
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
