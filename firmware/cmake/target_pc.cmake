set(TOOLCHAIN_PREFIX                )

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".out")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".out")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".out")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g0")
endif()

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")

set(CMAKE_C_LINK_FLAGS "")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS}")

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

set(BSP_PATH src/bsp/target-pc)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
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
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
)
