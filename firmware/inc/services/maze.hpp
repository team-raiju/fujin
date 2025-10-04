#pragma once

#include <cstdint>
#include <array>
#include <utility>
#include <vector>

#include "algorithms/flood_fill.hpp"
#include "navigation.hpp"
#include "utils/types.hpp"

namespace services {

class Maze {
public:
    static constexpr int CELLS_X = 16;
    static constexpr int CELLS_Y = 16;

    static constexpr Point ORIGIN = {0, 0};
    static constexpr std::array<Point, 1> ORIGIN_ARRAY = {{ORIGIN}};
    // static constexpr std::array<Point, 4> GOAL_POSITIONS = {{{8, 8}, {8, 7}, {7, 8}, {7, 7}}};
    static constexpr std::array<Point, 1> GOAL_POSITIONS = {{{8, 8}}};

    static Maze* instance();

    /// @brief This method receives information about a cell's walls and return
    ///        the next cell that should be visited
    /// @param current_position Current cell coordinates
    /// @param walls Current cell wall information
    /// @return Next cell to be visited
    Direction next_step(Point const& current_position, uint8_t walls, std::span<const Point> const& targets, bool search_mode = true);

    /// @brief Prints the maze for debugging purpose
    void print(Point const& curr);

    Maze(const Maze&) = delete;

    algorithm::Grid<CELLS_X, CELLS_Y> map;

    algorithm::Grid<CELLS_X, CELLS_Y> map_backup;

    std::vector<Direction> directions_to_goal();

    Point closest_unvisited(Point const& current_position);

    void save_maze_to_memory(bool backup);
    
    void read_maze_from_memory(bool backup);

    void create_maze_backup();

    void reset();

private:
    Maze();
};

}
