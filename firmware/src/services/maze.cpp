#include <cstdint>
#include <cstdio>
#include <cstring>
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
    this->reset();
}

void Maze::reset() {
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            map[x][y].walls = 0;
            map[x][y].known_walls = 0;
            map[x][y].distance = 255;
            map[x][y].north = nullptr;
            map[x][y].east = nullptr;
            map[x][y].south = nullptr;
            map[x][y].west = nullptr;

            map_backup[x][y].walls = 0;
            map_backup[x][y].known_walls = 0;
            map_backup[x][y].distance = 255;
            map_backup[x][y].north = nullptr;
            map_backup[x][y].east = nullptr;
            map_backup[x][y].south = nullptr;
            map_backup[x][y].west = nullptr;
        }
    }

    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            auto& cell = map[x][y];
            auto& cell_backup = map_backup[x][y];
            if (x > 0) {
                cell.west = &map[x - 1][y];
                cell_backup.west = &map_backup[x - 1][y];
            }

            if (x < CELLS_X - 1) {
                cell.east = &map[x + 1][y];
                cell_backup.east = &map_backup[x + 1][y];
            }

            if (y > 0) {
                cell.south = &map[x][y - 1];
                cell_backup.south = &map_backup[x][y - 1];
            }

            if (y < CELLS_Y - 1) {
                cell.north = &map[x][y + 1];
                cell_backup.north = &map_backup[x][y + 1];
            }

            // Populate outer walls
            if (x == 0) {
                cell.walls |= Walls::W;
                cell.known_walls |= Walls::W;
                cell_backup.walls |= Walls::W;
                cell_backup.known_walls |= Walls::W;
            }

            if (x == CELLS_X - 1) {
                cell.walls |= Walls::E;
                cell.known_walls |= Walls::E;
                cell_backup.walls |= Walls::E;
                cell_backup.known_walls |= Walls::E;
            }

            if (y == 0) {
                cell.walls |= Walls::S;
                cell.known_walls |= Walls::S;
                cell_backup.walls |= Walls::S;
                cell_backup.known_walls |= Walls::S;
            }

            if (y == CELLS_Y - 1) {
                cell.walls |= Walls::N;
                cell.known_walls |= Walls::N;
                cell_backup.walls |= Walls::N;
                cell_backup.known_walls |= Walls::N;
            }
        }
    }

    // We always start facing north, with walls to our back, left and right
    map[0][0].update_walls(Walls::E | Walls::W | Walls::S);
    map[0][0].known_walls = 0b1111;
}

Direction Maze::next_step(Point const& current_position, uint8_t walls, Point const& target, bool search_mode) {
    algorithm::Cell& cell = map[current_position.x][current_position.y];

    if (target == current_position) {
        if (search_mode) {
            cell.update_walls(walls);
        }
        return Direction::STOP;
    }

    // The only option in the origin is always north, don't waste time calculating
    if (current_position == ORIGIN) {
        return Direction::NORTH;
    }

    // Update our grid
    if (search_mode) {
        cell.update_walls(walls);
    }

    // Recalculate the distances
    algorithm::flood_fill(map, target, search_mode);

    if (map[current_position.x][current_position.y].distance == 255) {
        // Unreachable
        return Direction::STOP;
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
        if (!search_mode && !neighbour.visited()) {
            continue;
        }

        // TODO: Prioritize the current direction
        // We prioritize cells with smallest values and then unvisited cells if there's a tie
        tprio = neighbour.visited() ? 1 : 2;
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

Point Maze::closest_unvisited(Point const& current_position) {
    algorithm::flood_fill(map, current_position, true);

    int closest_dist = 255;
    auto closest_point = ORIGIN;
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            if (map[x][y].visited()) {
                continue;
            }

            if (map[x][y].distance < closest_dist) {
                closest_dist = map[x][y].distance;
                closest_point = {x, y};
            }
        }
    }

    return closest_point;
}

std::vector<Direction> Maze::directions_to_goal() {
    std::vector<Direction> target_directions = {};
    Point pos = {ORIGIN.x, ORIGIN.y + 1}; // start from the cell (0,1)

    bool goal_reached = false;

    while (!goal_reached) {
        auto dir = next_step(pos, map[pos.x][pos.y].walls, services::Maze::GOAL_POSITIONS[0], false);
        target_directions.push_back(dir);
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
        case Direction::STOP:
            return target_directions;
        }

        // If position is equal to any of the desired goals
        goal_reached = std::any_of(std::begin(GOAL_POSITIONS), std::end(GOAL_POSITIONS),
                                   [&](const Point& goal) { return goal == pos; });
    }

    return target_directions;
}

void Maze::save_maze_to_memory(bool backup) {
    uint8_t data[4];
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            auto& cell = backup ? map_backup[x][y] : map[x][y];
            data[0] = cell.walls;
            data[1] = cell.known_walls;
            data[2] = cell.distance;
            data[3] = 0;
            auto base_addr = backup ? bsp::eeprom::param_addresses_t::ADDR_MAZE_BACKUP_START
                                    : bsp::eeprom::param_addresses_t::ADDR_MAZE_START;
            bsp::eeprom::write_u32(base_addr + 4 * (x * CELLS_Y + y), *(uint32_t*)data);
            bsp::delay_ms(5);
        }
    }
}

void Maze::create_maze_backup() {
    std::memcpy(map_backup, map, sizeof(map));
}

void Maze::read_maze_from_memory(bool backup) {
    this->reset();
    uint8_t data[4];
    auto base_addr = backup ? bsp::eeprom::param_addresses_t::ADDR_MAZE_BACKUP_START
                            : bsp::eeprom::param_addresses_t::ADDR_MAZE_START;
    for (int x = 0; x < CELLS_X; x++) {
        for (int y = 0; y < CELLS_Y; y++) {
            bsp::eeprom::read_u32(base_addr + 4 * (x * CELLS_Y + y), (uint32_t*)data);
            map[x][y].walls = data[0];
            map[x][y].known_walls = data[1];
            map[x][y].distance = data[2];
            bsp::delay_ms(5);
        }
    }
}

void Maze::print(Point const& curr) {
    for (int y = (CELLS_Y - 1); y >= 0; y--) {
        // Top
        for (int x = 0; x < CELLS_X; x++) {
            std::printf(map[x][y].visited() ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & N ? "+---+" : "+   +");
        }

        std::printf("\r\n");
        bsp::delay_ms(2);

        // Left, value, right
        for (int x = 0; x < CELLS_X; x++) {
            std::printf(map[x][y].visited() ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & W ? "|" : " ");

            if (curr == Point{x, y}) {
                std::printf("\033[37m");
            }

            std::printf("%- 3d", map[x][y].distance);

            std::printf(map[x][y].visited() ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & E ? "|" : " ");
        }

        std::printf("\r\n");
        bsp::delay_ms(2);

        // Print bottom borders
        for (int x = 0; x < CELLS_X; x++) {
            std::printf(map[x][y].visited() ? "\033[33m" : "\033[34m");
            std::printf(map[x][y].walls & S ? "+---+" : "+   +");
        }

        std::printf("\r\n");
        std::printf("\033[39m");
        bsp::delay_ms(2);
    }
}

}
