#include <cstdint>
#include <cstdio>
#include <functional>
#include <queue>

#include "bsp/timers.hpp"
#include "services/maze.hpp"
#include "utils/RingBuffer.hpp"
#include "utils/math.hpp"

namespace services {

Maze* Maze::instance() {
    static Maze m;
    return &m;
}

Maze::Maze() {
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            auto& cell = map[x][y];
            if (x > 0) {
                cell.west = &map[x - 1][y];
            }

            if (x < CELLS_X - 1) {
                cell.east = &map[x + 1][y];
            }

            if (y > 0) {
                cell.south = &map[x][y - 1];
            }

            if (y < CELLS_Y - 1) {
                cell.north = &map[x][y + 1];
            }
        }
    }

    // We always start facing north, with walls to our back, left and right
    map[0][0].update_walls(Walls::E | Walls::W | Walls::S);
    map[0][0].visited = true;
}

Direction Maze::next_step(Point const& current_position, uint8_t walls, bool returning) {
    static RingBuffer<Point, 32> to_visit;

    // The only option in the origin is always north, don't waste time calculating
    if (current_position == ORIGIN) {
        return Direction::NORTH;
    }

    // Update our grid
    algorithm::Cell& cell = map[current_position.x][current_position.y];
    cell.update_walls(walls);
    cell.visited = true;

    // Recalculate the distances
    if (returning) {
        algorithm::flood_fill(map, ORIGIN);
    } else {
        algorithm::flood_fill(map, GOAL_POS);
    }

    // Check all directions for the best option
    static constexpr Point Δ[4] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};

    int prio = 0;
    int tprio;

    uint8_t smallest = 255;
    Direction next_direction = Direction::NORTH;
    for (auto& d : Directions) {
        // We can't go this direction, check next
        if ((cell.walls & (1 << d)) != 0) {
            continue;
        }

        Point position = current_position + Δ[std::to_underlying(d)];
        auto& neighbour = map[position.x][position.y];

        // out of bounds
        if (position.x < 0 || position.x >= CELLS_X || position.y < 0 || position.y >= CELLS_Y) {
            continue;
        }

        // TODO: Prioritize the current direction
        // We prioritize cells with smallest values and then unvisited cells if there's a tie
        tprio = neighbour.visited ? 1 : 2;
        if (neighbour.distance < smallest) {
            smallest = neighbour.distance;
            next_direction = d;
            prio = tprio;
        } else if (neighbour.distance == smallest && prio < tprio) {
            next_direction = d;
            prio = tprio;
        }
    }

    return next_direction;
}

void Maze::print(Point const& curr) {
    for (int y = (CELLS_Y - 1); y >= 0; y--) {
        // Top
        for (int x = 0; x < CELLS_X; x++) {
            std::printf(map[x][y].visited ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & N ? "+---+" : "+   +");
        }

        std::printf("\r\n");
        bsp::delay_ms(2);

        // Left, value, right
        for (int x = 0; x < CELLS_X; x++) {
            std::printf(map[x][y].visited ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & W ? "|" : " ");

            if (curr == Point{x, y}) {
                std::printf("\033[32m");
            }

            std::printf("%- 3d", map[x][y].distance);

            std::printf(map[x][y].visited ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & E ? "|" : " ");
        }

        std::printf("\r\n");
        bsp::delay_ms(2);

        // Print bottom borders
        for (int x = 0; x < CELLS_X; x++) {
            std::printf(map[x][y].visited ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & S ? "+---+" : "+   +");
        }

        std::printf("\r\n");
        std::printf("\033[39m");
        bsp::delay_ms(2);
    }
}

}
