#include <algorithm>
#include <array>
#include <cstdint>
#include <emscripten.h>
#include <span>
#include <utility>
#include <vector>

#include "algorithms/flood_fill.hpp"
#include "algorithms/movement_planner.hpp"
#include "algorithms/time_flood_fill.hpp"
#include "utils/movement_params.hpp"
#include "utils/types.hpp"

using namespace algorithm;

namespace {
Grid<16, 16> grid;
std::vector<Direction> current_path_dirs;
std::vector<std::pair<Movement, uint8_t>> current_movements;
float current_estimated_time_s = 0.0f;

uint16_t calculate_step_cost_local(
    CompassHeading curr_heading, CompassHeading next_heading,
    uint8_t curr_run_count, uint8_t next_run_count,
    uint16_t turn_90_ms, uint16_t turn_90_diag_ms,
    const CellWeight* straight_weights,
    const CellWeight* diagonal_weights) {

    bool is_ortho_curr = is_orthogonal(curr_heading);
    bool is_ortho_next = is_orthogonal(next_heading);
    bool is_diag_curr  = is_diagonal(curr_heading);
    bool is_diag_next  = is_diagonal(next_heading);

    if (is_ortho_curr && is_ortho_next) {
        if (curr_heading == next_heading) {
            return static_cast<uint16_t>(straight_weights[next_run_count].time_ms);
        } else {
            float penalty = straight_weights[curr_run_count].penalty_ms;
            float step_time = straight_weights[0].time_ms;
            return static_cast<uint16_t>(penalty + turn_90_ms + step_time);
        }
    } else if (is_diag_curr && is_diag_next) {
        if (curr_heading == next_heading) {
            return static_cast<uint16_t>(diagonal_weights[next_run_count].time_ms);
        } else {
            float penalty = diagonal_weights[curr_run_count].penalty_ms;
            float step_time = diagonal_weights[0].time_ms;
            return static_cast<uint16_t>(penalty + turn_90_diag_ms + step_time);
        }
    } else if (is_ortho_curr && is_diag_next) {
        float penalty = straight_weights[curr_run_count].penalty_ms;
        float step_time = diagonal_weights[0].time_ms;
        return static_cast<uint16_t>(penalty + step_time);
    } else if (is_diag_curr && is_ortho_next) {
        float penalty = diagonal_weights[curr_run_count].penalty_ms;
        float step_time = straight_weights[0].time_ms;
        return static_cast<uint16_t>(penalty + step_time);
    }

    return static_cast<uint16_t>(straight_weights[0].time_ms);
}

float calculate_path_time_s(
    std::span<const Direction> path,
    const std::array<ForwardParams, MOVEMENT_COUNT>& fwd_params,
    const std::array<TurnParams, MOVEMENT_COUNT>& trn_params) {

    if (path.empty()) {
        return 0.0f;
    }

    float max_speed = fwd_params[Movement::FORWARD].max_speed;
    float accel = fwd_params[Movement::FORWARD].acceleration;
    float decel = fwd_params[Movement::FORWARD].deceleration;
    float init_speed = trn_params[Movement::TURN_RIGHT_90].turn_linear_speed;

    if (init_speed <= 0.1f) init_speed = 0.5f;
    if (max_speed < init_speed) max_speed = init_speed;
    if (accel < 0.1f) accel = 1.0f;
    if (decel < 0.1f) decel = 1.0f;

    float diag_max_speed = fwd_params[Movement::DIAGONAL].max_speed;
    float diag_accel = fwd_params[Movement::DIAGONAL].acceleration;
    float diag_decel = fwd_params[Movement::DIAGONAL].deceleration;

    if (diag_max_speed <= 0.1f) diag_max_speed = max_speed * 0.85f;
    if (diag_accel <= 0.1f) diag_accel = accel;
    if (diag_decel <= 0.1f) diag_decel = decel;

    CellWeight straight_weights[TimeFloodFill::MAX_WEIGHT_STEPS];
    CellWeight diagonal_weights[TimeFloodFill::MAX_WEIGHT_STEPS];

    TimeFloodFill::generate_kinematic_weights(0.18f, init_speed, max_speed, accel, decel, TimeFloodFill::MAX_WEIGHT_STEPS, straight_weights);
    TimeFloodFill::generate_kinematic_weights(0.12728f, init_speed, diag_max_speed, diag_accel, diag_decel, TimeFloodFill::MAX_WEIGHT_STEPS, diagonal_weights);

    uint16_t turn_90_ms = 120;
    uint16_t turn_90_diag_ms = 90;

    CompassHeading curr_heading = CompassHeading::NORTH;
    Direction last_step = Direction::NORTH;
    uint8_t curr_run = 1;
    uint32_t total_cost_ms = 0;

    for (Direction next_dir : path) {
        CompassHeading new_h = TimeFloodFill::get_next_heading(curr_heading, last_step, next_dir);
        uint8_t next_run = (new_h == curr_heading)
            ? static_cast<uint8_t>(std::min<size_t>(curr_run + 1, TimeFloodFill::MAX_WEIGHT_STEPS - 1))
            : 0;

        uint16_t step_cost = calculate_step_cost_local(
            curr_heading,
            new_h,
            curr_run,
            next_run,
            turn_90_ms,
            turn_90_diag_ms,
            straight_weights,
            diagonal_weights);

        total_cost_ms += step_cost;
        curr_heading = new_h;
        curr_run = next_run;
        last_step = next_dir;
    }

    return total_cost_ms / 1000.0f;
}
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
            if (h_walls[r * 16 + c])
                w |= Walls::N;
            if (h_walls[(r + 1) * 16 + c])
                w |= Walls::S;
            if (v_walls[r * 17 + c])
                w |= Walls::W;
            if (v_walls[r * 17 + c + 1])
                w |= Walls::E;
            grid[x][y].walls = w;
        }
    }
}

EMSCRIPTEN_KEEPALIVE
void wasm_run_flood_fill(const Point* goals, int num_goals, int search_mode) {
    flood_fill(grid, std::span<const Point>(goals, num_goals), search_mode != 0);
}

EMSCRIPTEN_KEEPALIVE
uint8_t wasm_get_cell_distance(int x, int y) {
    if (x < 0 || x >= 16 || y < 0 || y >= 16)
        return 255;
    return grid[x][y].distance;
}

EMSCRIPTEN_KEEPALIVE
int wasm_run_time_flood_fill(int start_x, int start_y, const Point* goals, int num_goals, int speed_mode = 5) {
    Point start{start_x, start_y};
    std::span<const Point> goal_span(goals, num_goals);

    navigation_mode_t nav_mode = static_cast<navigation_mode_t>(speed_mode);
    const auto& fwd = get_forward_params(nav_mode);
    const auto& trn = get_turn_params(nav_mode);

    current_path_dirs = TimeFloodFill::find_fastest_path(grid, start, goal_span, fwd, trn, &current_estimated_time_s);

    return static_cast<int>(current_path_dirs.size());
}

EMSCRIPTEN_KEEPALIVE
float wasm_get_estimated_time_s() {
    return current_estimated_time_s;
}

EMSCRIPTEN_KEEPALIVE
int wasm_trace_classic_path(int start_x, int start_y, const Point* goals, int num_goals, int speed_mode = 5) {
    current_path_dirs.clear();
    current_estimated_time_s = 0.0f;

    std::span<const Point> goal_span(goals, num_goals);
    flood_fill(grid, goal_span, 0);

    Point curr{start_x, start_y};
    static constexpr Point Δ[4] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};

    for (int step = 0; step < 512; ++step) {
        bool at_goal = false;
        for (const auto& g : goal_span) {
            if (curr == g) {
                at_goal = true;
                break;
            }
        }
        if (at_goal) {
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

    navigation_mode_t nav_mode = static_cast<navigation_mode_t>(speed_mode);
    const auto& fwd = get_forward_params(nav_mode);
    const auto& trn = get_turn_params(nav_mode);
    current_estimated_time_s = calculate_path_time_s(current_path_dirs, fwd, trn);

    return static_cast<int>(current_path_dirs.size());
}

EMSCRIPTEN_KEEPALIVE
int wasm_compute_movements(int start_x, int start_y, int movement_mode) {
    (void)start_x;
    (void)start_y;
    std::vector<Direction> planner_dirs;
    if (!current_path_dirs.empty()) {
        planner_dirs.assign(current_path_dirs.begin() + 1, current_path_dirs.end());
    }
    current_movements = MovementPlanner::plan_movements(planner_dirs, static_cast<MovementMode>(movement_mode));
    return static_cast<int>(current_movements.size());
}

EMSCRIPTEN_KEEPALIVE
int wasm_get_movement_count(int index) {
    if (index < 0 || static_cast<size_t>(index) >= current_movements.size())
        return 0;
    return static_cast<int>(current_movements[index].second);
}

EMSCRIPTEN_KEEPALIVE
int wasm_get_movement_type(int index) {
    if (index < 0 || static_cast<size_t>(index) >= current_movements.size())
        return -1;
    return static_cast<int>(current_movements[index].first);
}

EMSCRIPTEN_KEEPALIVE
int wasm_get_path_dir(int index) {
    if (index < 0 || static_cast<size_t>(index) >= current_path_dirs.size())
        return -1;
    return static_cast<int>(current_path_dirs[index]);
}

} // extern "C"
