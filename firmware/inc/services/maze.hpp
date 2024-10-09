#pragma once

#include <cstdint>
#include <array>

#include "algorithms/flood_fill.hpp"
#include "navigation.hpp"

namespace services {

class Maze {
public:
    static constexpr int CELLS_X = 4;
    static constexpr int CELLS_Y = 7;

    static constexpr Point ORIGIN = {0, 0};
    static constexpr Point GOAL_POS = {1, 5};

    static Maze* instance();

    /// @brief This method receives information about a cell's walls and return
    ///        the next cell that should be visited
    /// @param current_position Current cell coordinates
    /// @param walls Current cell wall information
    /// @return Next cell to be visited
    Direction next_step(Point const& current_position, uint8_t walls, bool returning = false, bool search_mode = true);

    /// @brief Prints the maze for debugging purpose
    void print(Point const& curr);

    Maze(const Maze&) = delete;

    algorithm::Grid<CELLS_X, CELLS_Y> map;

    std::array<Direction, 256> directions_to_goal();

    void save_maze_to_memory();
    
    void read_maze_from_memory();

private:
    Maze();
};

}
