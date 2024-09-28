#pragma once

#include <cstdint>
#include "utils/navigation_types.hpp"

using namespace navigation;

class Maze {
public:
    static constexpr int MAX_CELLS_X = 4;
    static constexpr int MAX_CELLS_Y = 7;
    static constexpr int MAX_STEP_VALUE = 255;
    static constexpr int GOAL_X_POS = 0;
    static constexpr int GOAL_Y_POS = 6;
    static constexpr float CELL_SIZE_CM = 18.0;
    static constexpr float HALF_CELL_SIZE_CM = 9.0;

    enum StepMapType {
        SEARCH,
        RUN,
    };

    enum WallInfo {
        NO_WALL,
        WALL,
        UNKNOWN_WALL,
    };

    struct Cell {
        WallInfo north : 2;
        WallInfo east : 2;
        WallInfo south : 2;
        WallInfo west : 2;
    };

    Maze();

    /// @brief  Initialize the step map
    void init_step_map(Position goal);

    /// @brief  Calculate the step map using flood fill algorithm
    void calculate_step_map(Position goal, StepMapType type);

    /// @brief Update wall map based on current sensor readings
    void update_wall_map(Position robot_pos, Direction robot_dir, bool front_seeing, bool right_seeing,
                         bool left_seeing);

    Direction get_target_cell_dir(Position robot_pos, Direction robot_dir, StepMapType type);

    Movement get_target_movement(Direction robot_dir, Direction target_dir);

    /// @brief prints wall map
    void print_maze();

private:
    bool cell_is_valid(Position pos);
    int get_cell_priority(Direction robot_dir, Position desired_pos, Direction desired_dir) ;
    bool cell_is_unexplored(Position pos);
    Cell wall_map[MAX_CELLS_X][MAX_CELLS_Y];
    uint8_t step_map[MAX_CELLS_X][MAX_CELLS_Y];
};
