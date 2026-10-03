#include <emscripten.h>
#include <cstdint>
#include <vector>
#include <array>
#include <utility>

#include "algorithms/flood_fill.hpp"
#include "algorithms/time_flood_fill.hpp"
#include "algorithms/movement_planner.hpp"
#include "utils/movement_params.hpp"
#include "utils/types.hpp"

using namespace algorithm;

namespace {
    Grid<16, 16> grid;
    std::vector<Direction> current_path_dirs;
    std::vector<std::pair<Movement, uint8_t>> current_movements;
    float current_estimated_time_s = 0.0f;
}

extern "C" {

EMSCRIPTEN_KEEPALIVE
void wasm_init_grid() {
    for (int x = 0; x < 16; ++x) {
        for (int y = 0; y < 16; ++y) {
            grid[x][y].distance = 255;
            grid[x][y].walls = 0;
            grid[x][y].known_walls = 0b1111;
            grid[x][y].north = (y < 15) ? &grid[x][y + 1] : nullptr;
            grid[x][y].south = (y > 0) ? &grid[x][y - 1] : nullptr;
            grid[x][y].east = (x < 15) ? &grid[x + 1][y] : nullptr;
            grid[x][y].west = (x > 0) ? &grid[x - 1][y] : nullptr;
        }
    }
}

EMSCRIPTEN_KEEPALIVE
void wasm_set_walls_from_arrays(const uint8_t* h_walls, const uint8_t* v_walls) {
    wasm_init_grid();
    for (int x = 0; x < 16; ++x) {
        for (int y = 0; y < 16; ++y) {
            int r = 15 - y;
            int c = x;
            uint8_t w = 0;
            if (h_walls[r * 16 + c]) w |= Walls::N;
            if (h_walls[(r + 1) * 16 + c]) w |= Walls::S;
            if (v_walls[r * 17 + c]) w |= Walls::W;
            if (v_walls[r * 17 + c + 1]) w |= Walls::E;
            grid[x][y].walls = w;
        }
    }
}

EMSCRIPTEN_KEEPALIVE
void wasm_run_flood_fill(int target_x, int target_y, int search_mode) {
    flood_fill(grid, {target_x, target_y}, search_mode != 0);
}

EMSCRIPTEN_KEEPALIVE
uint8_t wasm_get_cell_distance(int x, int y) {
    if (x < 0 || x >= 16 || y < 0 || y >= 16) return 255;
    return grid[x][y].distance;
}

EMSCRIPTEN_KEEPALIVE
int wasm_run_time_flood_fill(int start_x, int start_y, int goal_x, int goal_y, int speed_mode = 0) {
    (void)speed_mode;
    Point start{start_x, start_y};
    std::vector<Point> goals;
    if (goal_x < 0) {
        goals = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    } else {
        goals = {{goal_x, goal_y}};
    }

    current_path_dirs = TimeFloodFill::find_fastest_path(
        grid, start, goals, forward_params_custom, turn_params_custom, &current_estimated_time_s);

    return static_cast<int>(current_path_dirs.size());
}

EMSCRIPTEN_KEEPALIVE
float wasm_get_estimated_time_s() {
    return current_estimated_time_s;
}

EMSCRIPTEN_KEEPALIVE
int wasm_trace_classic_path(int start_x, int start_y, int goal_x, int goal_y) {
    current_path_dirs.clear();
    current_estimated_time_s = 0.0f;

    wasm_run_flood_fill(goal_x, goal_y, 0);

    Point curr{start_x, start_y};
    static constexpr Point Δ[4] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};

    for (int step = 0; step < 512; ++step) {
        if (curr.x == goal_x && curr.y == goal_y) {
            break;
        }

        uint8_t min_dist = grid[curr.x][curr.y].distance;
        Direction best_dir = Direction::STOP;
        Point best_next = curr;

        for (auto d : Directions) {
            uint8_t d_idx = std::to_underlying(d);
            if ((grid[curr.x][curr.y].walls & (1 << d_idx)) != 0) {
                continue; // wall blocked
            }
            Point next = curr + Δ[d_idx];
            if (next.x < 0 || next.x >= 16 || next.y < 0 || next.y >= 16) {
                continue;
            }
            if (grid[next.x][next.y].distance < min_dist) {
                min_dist = grid[next.x][next.y].distance;
                best_dir = d;
                best_next = next;
            }
        }

        if (best_dir == Direction::STOP) {
            break;
        }

        current_path_dirs.push_back(best_dir);
        curr = best_next;
    }

    return static_cast<int>(current_path_dirs.size());
}

EMSCRIPTEN_KEEPALIVE
int wasm_compute_movements(int start_x, int start_y, int movement_mode) {
    (void)start_x;
    (void)start_y;
    current_movements = MovementPlanner::plan_movements(
        current_path_dirs, static_cast<MovementMode>(movement_mode));
    return static_cast<int>(current_movements.size());
}

EMSCRIPTEN_KEEPALIVE
int wasm_get_movement_count(int index) {
    if (index < 0 || static_cast<size_t>(index) >= current_movements.size()) return 0;
    return static_cast<int>(current_movements[index].second);
}

EMSCRIPTEN_KEEPALIVE
int wasm_get_movement_type(int index) {
    if (index < 0 || static_cast<size_t>(index) >= current_movements.size()) return -1;
    return static_cast<int>(current_movements[index].first);
}

EMSCRIPTEN_KEEPALIVE
int wasm_get_path_dir(int index) {
    if (index < 0 || static_cast<size_t>(index) >= current_path_dirs.size()) return -1;
    return static_cast<int>(current_path_dirs[index]);
}

} // extern "C"
