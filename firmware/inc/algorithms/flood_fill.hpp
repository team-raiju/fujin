#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

#include "utils/RingBuffer.hpp"
#include "utils/math.hpp"
#include "utils/types.hpp"

enum Walls : uint8_t {
    N = 1 << Direction::NORTH,
    W = 1 << Direction::WEST,
    E = 1 << Direction::EAST,
    S = 1 << Direction::SOUTH,
};

namespace algorithm {

struct Cell {
    uint8_t distance;
    uint8_t walls;
    uint8_t known_walls;

    bool visited() { return known_walls == 0b1111; }

    Cell* north;
    Cell* east;
    Cell* south;
    Cell* west;

    void update_walls(uint8_t walls) {
        this->walls = walls;
        this->known_walls = 0b1111;

        if (walls & N && (north != nullptr)) {
            north->walls |= S;
            north->known_walls |= S;
        } else if (north != nullptr) {
            north->walls &= ~S;
            north->known_walls |= S;
        }

        if (walls & E && (east != nullptr)) {
            east->walls |= W;
            east->known_walls |= W;
        } else if (east != nullptr) {
            east->walls &= ~W;
            east->known_walls |= W;
        }

        if (walls & S && (south != nullptr)) {
            south->walls |= N;
            south->known_walls |= N;
        } else if (south != nullptr) {
            south->walls &= ~N;
            south->known_walls |= N;
        }

        if (walls & W && (west != nullptr)) {
            west->walls |= E;
            west->known_walls |= E;
        } else if (west != nullptr) {
            west->walls &= ~E;
            west->known_walls |= E;
        }
    }
};

template <int width, int height>
using Grid = Cell[width][height];

template <int width, int height>
void flood_fill(Grid<width, height>& grid, Point const& target, bool search_mode = true) {
    // 1. Reset the distance of every cell
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            grid[x][y].distance = 255;
        }
    }

    RingBuffer<Point, 32> to_visit;

    grid[target.x][target.y].distance = 0;
    to_visit.put(target);

    static constexpr Point Δ[4] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};

    while (!to_visit.empty()) {
        Point pos = {0, 0};
        to_visit.get(&pos);
        Cell& cell = grid[pos.x][pos.y];

        // Iterate every direction
        for (auto& d : Directions) {
            Point next = pos + Δ[std::to_underlying(d)];

            // out of bounds
            if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height) {
                continue;
            }

            Cell& next_cell = grid[next.x][next.y];

            bool has_wall = (cell.walls & (1 << d)) != 0;
            bool visited = next_cell.distance != 255;

            // We can't or don't need to visit this cell
            if (has_wall || visited) {
                continue;
            }

            // We only visit cells that have not been visited before in search mode
            if (!search_mode) {
                if (!next_cell.visited()) {
                    continue;
                }
            }

            next_cell.distance = cell.distance + 1;
            to_visit.put(next);
        }
    }
}

}
