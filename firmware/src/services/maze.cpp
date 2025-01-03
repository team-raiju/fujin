#include <cstdint>
#include <cstdio>
#include <functional>
#include <queue>
#include <utility>

#include "bsp/eeprom.hpp"
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

Direction Maze::next_step(Point const& current_position, uint8_t walls, bool returning, bool search_mode) {
    static RingBuffer<Point, 32> to_visit;

    // The only option in the origin is always north, don't waste time calculating
    if (current_position == ORIGIN) {
        return Direction::NORTH;
    }

    // Update our grid
    algorithm::Cell& cell = map[current_position.x][current_position.y];
    if (search_mode) {
        cell.update_walls(walls);
        cell.visited = true;
    }

    // Recalculate the distances
    if (returning) {
        algorithm::flood_fill(map, ORIGIN, search_mode);
    } else {
        algorithm::flood_fill(map, GOAL_POS, search_mode);
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

        // out of bounds
        if (position.x < 0 || position.x >= CELLS_X || position.y < 0 || position.y >= CELLS_Y) {
            continue;
        }

        auto& neighbour = map[position.x][position.y];

        // In run mode, we don't visit cells that have not been visited before
        if (!search_mode && !neighbour.visited) {
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

std::array<std::pair<Direction, uint8_t>, 256> Maze::directions_to_goal() {
    // std::array<Direction, 256> target_directions = {};
    std::array<std::pair<Direction, uint8_t>, 256> processed_directions = {};
    Point pos = ORIGIN;
    pos.y += 1; // start from the cell (0,1)

/*     int dir_count = 0;

    while (pos != GOAL_POS) {
        auto dir = next_step(pos, map[pos.x][pos.y].walls, false, false);
        target_directions[dir_count++] = dir;
        switch (dir) {
        case Direction::NORTH:
            pos.y += 1;
            break;
        case Direction::EAST:
            pos.x += 1;
            break;
        case Direction::SOUTH:
            pos.y -= 1;
            break;
        case Direction::WEST:
            pos.x -= 1;
            break;
        }
    }

    uint8_t continuous_cnt = 1;
    int processed_count = 1;
    processed_directions[0] = {target_directions[0], 1};
    processed_directions[1] = {target_directions[1], 1};

    for (int i = 2; i < dir_count; i++) {
        if (target_directions[i] == target_directions[i - 1] && target_directions[i] == target_directions[i - 2]) {
            continuous_cnt++;
        } else {
            processed_directions[processed_count++] = {target_directions[i - 1], continuous_cnt};
            continuous_cnt = 1;
        }
    }
    processed_directions[processed_count] = {target_directions[dir_count - 1], continuous_cnt}; */

    std::array<std::pair<Direction, uint8_t>, 1> process_direction = {
        std::make_pair(Direction::EAST, 1)
    };

    for (size_t i = 0; i < process_direction.size(); ++i) {
        processed_directions[i] = process_direction[i];
    }

    return processed_directions;
}

void Maze::save_maze_to_memory() {
    uint8_t data[4];
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            auto& cell = map[x][y];
            data[0] = cell.walls;
            data[1] = cell.visited;
            data[2] = cell.distance;
            data[3] = 0;
            bsp::eeprom::write_u32(bsp::eeprom::param_addresses_t::ADDR_MAZE_START + 4 * (x * CELLS_Y + y),
                                   *(uint32_t*)data);
            bsp::delay_ms(5);
        }
    }
}

void Maze::read_maze_from_memory() {
    uint8_t data[4];
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            bsp::eeprom::read_u32(bsp::eeprom::param_addresses_t::ADDR_MAZE_START + 4 * (x * CELLS_Y + y),
                                  (uint32_t*)data);
            map[x][y].walls = data[0];
            map[x][y].visited = data[1];
            map[x][y].distance = data[2];
            bsp::delay_ms(5);
        }
    }
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
                std::printf("\033[37m");
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
