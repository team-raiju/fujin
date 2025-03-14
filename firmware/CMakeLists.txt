cmake_minimum_required(VERSION 3.22)


#### Setup compiler settings ####
set(CMAKE_C_STANDARD                23)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              23)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()


#### Project ####
project(fujin)
add_executable(${CMAKE_PROJECT_NAME})
message("CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

if(PC_BUILD)
    include("cmake/target_pc.cmake")
else()
    include("cmake/target_stm32g4.cmake")
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    src/main.cpp

    src/fsm/fsm.cpp
    src/fsm/states/idle.cpp
    src/fsm/states/run.cpp
    src/fsm/states/search.cpp
    src/fsm/states/calib.cpp

    src/algorithms/pid.cpp

    src/utils/soft_timer.cpp

    src/services/navigation.cpp
    src/services/control.cpp
    src/services/maze.cpp
    src/services/notification.cpp
    src/services/logger.cpp

    src/services/config.cpp

)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    inc
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include 'utils/types.hpp' ")

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
    TARGET=${TARGET}
)
