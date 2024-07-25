cmake_minimum_required(VERSION 3.22)


#### Setup compiler settings ####
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: "              ${CMAKE_BUILD_TYPE})


#### Project ####
project(fujin)

enable_language(C CXX ASM)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

include(toolchain_stm32.cmake)

if (RELEASE_BUILD STREQUAL "true")
    list(APPEND SYMBOLS_SYMB
        "RELEASE"
    )
endif()


# Executable files
add_executable(${EXECUTABLE} ${SRC_FILES})

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${INCLUDE_DIRS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${SYMBOLS_SYMB})

# Print compiler flags
message(STATUS "CFLAGS: ${CFLAGS}")
message(STATUS "OPT FLAGS": ${OPT_LEVEL})
message(STATUS "SYMBOLS_SYMB: ${SYMBOLS_SYMB}")

#### Compiler options ####
target_compile_options(${EXECUTABLE} PRIVATE
    ${CFLAGS}
)

target_link_libraries(${EXECUTABLE} PRIVATE
    ${SPECIFIC_LIBS}
    m #Add libm (-lm)
)

target_link_options(${EXECUTABLE} PRIVATE
    ${LINKER_FLAGS}
)


# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)

# ### List of files to format ####
file(GLOB_RECURSE FILES_TO_FORMAT
    ${PROJ_PATH}/src/*.c
    ${PROJ_PATH}/inc/*.h
)

# Format files
add_custom_target(format-files
COMMAND clang-format -i ${FILES_TO_FORMAT}
WORKING_DIRECTORY ${PROJ_PATH})
