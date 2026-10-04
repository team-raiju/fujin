set(CMAKE_EXECUTABLE_SUFFIX ".wasm")

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "fujin_algorithms"
    SUFFIX ".wasm"
    LINK_FLAGS "-s STANDALONE_WASM=1 --no-entry -O3"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    src/wasm/fujin_algorithms.cpp
    src/algorithms/time_flood_fill.cpp
    src/algorithms/movement_planner.cpp
    src/utils/movement_params.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    inc
    src
)
