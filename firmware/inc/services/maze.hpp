#pragma once

#include <cstdint>

#include "navigation.hpp"

namespace services {

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

    static Maze* instance();

    /// @brief  Initialize the step map
    void init_step_map(Point goal);

    /// @brief  Calculate the step map using flood fill algorithm
    void calculate_step_map(Point goal, StepMapType type);

    /// @brief Update wall map based on current sensor readings
    void update_wall_map(Point robot_pos, Direction robot_dir, bool front_seeing, bool right_seeing, bool left_seeing);

    Direction get_target_cell_dir(Point robot_pos, Direction robot_dir, StepMapType type);

    Movement get_target_movement(Direction robot_dir, Direction target_dir);

    /// @brief prints wall map
    void print_maze();

    Maze(const Maze&) = delete;

private:
    Maze();

    bool cell_is_valid(Point pos);
    int get_cell_priority(Direction robot_dir, Point desired_pos, Direction desired_dir);
    bool cell_is_unexplored(Point pos);
    Cell wall_map[MAX_CELLS_X][MAX_CELLS_Y];
    uint8_t step_map[MAX_CELLS_X][MAX_CELLS_Y];
};

}
