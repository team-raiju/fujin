#include "algorithms/time_flood_fill.hpp"

#include <cmath>
#include <algorithm>

namespace algorithm {

namespace {
    uint16_t cost_matrix[16][16][8];
    PackedParent parent_matrix[16][16][8];
    MinHeap<QueueItem, 256> heap;
    CellWeight straight_weights[TimeFloodFill::MAX_WEIGHT_STEPS];
    CellWeight diagonal_weights[TimeFloodFill::MAX_WEIGHT_STEPS];
}

void TimeFloodFill::generate_kinematic_weights(
    float cell_dist, float init_speed, float max_speed, float accel, float decel,
    size_t count, CellWeight* weights_out) {

    float v = init_speed;
    if (v < 0.1f) v = 0.5f;
    if (accel < 0.1f) accel = 1.0f;
    if (decel < 0.1f) decel = 1.0f;
    if (max_speed < v) max_speed = v;

    for (size_t i = 0; i < count; ++i) {
        float time = 0.0f;
        float v_next = v;

        if (v >= max_speed) {
            v_next = max_speed;
            time = cell_dist / max_speed;
        } else {
            float d_to_max = (max_speed * max_speed - v * v) / (2.0f * accel);
            if (cell_dist <= d_to_max) {
                v_next = std::sqrt(v * v + 2.0f * accel * cell_dist);
                if (v_next > max_speed) v_next = max_speed;
                time = (v_next - v) / accel;
            } else {
                float t_accel = (max_speed - v) / accel;
                float d_cruise = cell_dist - d_to_max;
                float t_cruise = d_cruise / max_speed;
                time = t_accel + t_cruise;
                v_next = max_speed;
            }
        }

        float penalty = 0.0f;
        if (v_next > init_speed) {
            penalty = (v_next - init_speed) / decel;
        }

        weights_out[i].time_ms = time * 1000.0f;
        weights_out[i].penalty_ms = penalty * 1000.0f;

        v = v_next;
    }
}

CompassHeading TimeFloodFill::get_next_heading(
    CompassHeading curr_heading, Direction last_step, Direction next_step) {

    switch (curr_heading) {
    case CompassHeading::NONE:
        return to_compass(next_step);

    case CompassHeading::NORTH:
        switch (next_step) {
        case Direction::NORTH: return CompassHeading::NORTH;
        case Direction::EAST:  return CompassHeading::NORTH_EAST;
        case Direction::WEST:  return CompassHeading::NORTH_WEST;
        case Direction::SOUTH: return CompassHeading::SOUTH;
        default:               return CompassHeading::NORTH;
        }

    case CompassHeading::EAST:
        switch (next_step) {
        case Direction::EAST:  return CompassHeading::EAST;
        case Direction::NORTH: return CompassHeading::NORTH_EAST;
        case Direction::SOUTH: return CompassHeading::SOUTH_EAST;
        case Direction::WEST:  return CompassHeading::WEST;
        default:               return CompassHeading::EAST;
        }

    case CompassHeading::SOUTH:
        switch (next_step) {
        case Direction::SOUTH: return CompassHeading::SOUTH;
        case Direction::EAST:  return CompassHeading::SOUTH_EAST;
        case Direction::WEST:  return CompassHeading::SOUTH_WEST;
        case Direction::NORTH: return CompassHeading::NORTH;
        default:               return CompassHeading::SOUTH;
        }

    case CompassHeading::WEST:
        switch (next_step) {
        case Direction::WEST:  return CompassHeading::WEST;
        case Direction::NORTH: return CompassHeading::NORTH_WEST;
        case Direction::SOUTH: return CompassHeading::SOUTH_WEST;
        case Direction::EAST:  return CompassHeading::EAST;
        default:               return CompassHeading::WEST;
        }

    case CompassHeading::NORTH_EAST:
        switch (next_step) {
        case Direction::NORTH:
            return (last_step == Direction::EAST) ? CompassHeading::NORTH_EAST : CompassHeading::NORTH;
        case Direction::EAST:
            return (last_step == Direction::NORTH) ? CompassHeading::NORTH_EAST : CompassHeading::EAST;
        case Direction::SOUTH:
            return CompassHeading::SOUTH_EAST;
        case Direction::WEST:
            return CompassHeading::NORTH_WEST;
        default:
            return CompassHeading::NORTH_EAST;
        }

    case CompassHeading::SOUTH_EAST:
        switch (next_step) {
        case Direction::SOUTH:
            return (last_step == Direction::EAST) ? CompassHeading::SOUTH_EAST : CompassHeading::SOUTH;
        case Direction::EAST:
            return (last_step == Direction::SOUTH) ? CompassHeading::SOUTH_EAST : CompassHeading::EAST;
        case Direction::NORTH:
            return CompassHeading::NORTH_EAST;
        case Direction::WEST:
            return CompassHeading::SOUTH_WEST;
        default:
            return CompassHeading::SOUTH_EAST;
        }

    case CompassHeading::SOUTH_WEST:
        switch (next_step) {
        case Direction::SOUTH:
            return (last_step == Direction::WEST) ? CompassHeading::SOUTH_WEST : CompassHeading::SOUTH;
        case Direction::WEST:
            return (last_step == Direction::SOUTH) ? CompassHeading::SOUTH_WEST : CompassHeading::WEST;
        case Direction::NORTH:
            return CompassHeading::NORTH_WEST;
        case Direction::EAST:
            return CompassHeading::SOUTH_EAST;
        default:
            return CompassHeading::SOUTH_WEST;
        }

    case CompassHeading::NORTH_WEST:
        switch (next_step) {
        case Direction::NORTH:
            return (last_step == Direction::WEST) ? CompassHeading::NORTH_WEST : CompassHeading::NORTH;
        case Direction::WEST:
            return (last_step == Direction::NORTH) ? CompassHeading::NORTH_WEST : CompassHeading::WEST;
        case Direction::SOUTH:
            return CompassHeading::SOUTH_WEST;
        case Direction::EAST:
            return CompassHeading::NORTH_EAST;
        default:
            return CompassHeading::NORTH_WEST;
        }
    }

    return to_compass(next_step);
}

uint16_t TimeFloodFill::calculate_step_cost(
    CompassHeading curr_heading, CompassHeading next_heading,
    uint8_t curr_run_count, uint8_t next_run_count,
    uint16_t turn_90_ms, uint16_t turn_90_diag_ms) {

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

std::vector<Direction> TimeFloodFill::find_fastest_path(
    const Grid<16, 16>& grid,
    Point start_pos,
    std::span<const Point> goals,
    const std::array<ForwardParams, MOVEMENT_COUNT>& fwd_params,
    const std::array<TurnParams, MOVEMENT_COUNT>& trn_params,
    float* out_estimated_time_s) {

    // 1. Initialize matrices and heap
    std::fill_n(&cost_matrix[0][0][0], 16 * 16 * 8, 0xFFFF);
    std::fill_n(reinterpret_cast<uint16_t*>(&parent_matrix[0][0][0]), 16 * 16 * 8, 0);
    heap.clear();

    // 2. Extract kinematic parameters
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

    generate_kinematic_weights(0.18f, init_speed, max_speed, accel, decel, MAX_WEIGHT_STEPS, straight_weights);
    generate_kinematic_weights(0.12728f, init_speed, diag_max_speed, diag_accel, diag_decel, MAX_WEIGHT_STEPS, diagonal_weights);

    uint16_t turn_90_ms = 120;
    uint16_t turn_90_diag_ms = 90;

    // 3. Seed initial state at start_pos facing NORTH
    CompassHeading start_heading = CompassHeading::NORTH;
    uint8_t start_h_idx = static_cast<uint8_t>(start_heading);
    cost_matrix[start_pos.x][start_pos.y][start_h_idx] = 0;

    heap.push({
        0,
        static_cast<uint8_t>(start_pos.x),
        static_cast<uint8_t>(start_pos.y),
        start_h_idx,
        1,
        Direction::NORTH
    });

    bool found_goal = false;
    QueueItem goal_state{};

    static constexpr Point Δ[4] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};

    // 4. Dijkstra exploration
    while (!heap.empty()) {
        QueueItem curr = heap.pop();
        uint8_t h_idx = curr.heading;

        if (curr.cost_ms > cost_matrix[curr.x][curr.y][h_idx]) {
            continue;
        }

        Point current_pt = {curr.x, curr.y};

        // Check if goal reached
        for (const auto& g : goals) {
            if (current_pt == g) {
                found_goal = true;
                goal_state = curr;
                break;
            }
        }
        if (found_goal) {
            break;
        }

        const auto& current_cell = grid[curr.x][curr.y];

        for (auto next_dir : Directions) {
            uint8_t dir_idx = std::to_underlying(next_dir);
            Point next_pt = current_pt + Δ[dir_idx];

            if (next_pt.x < 0 || next_pt.x >= 16 || next_pt.y < 0 || next_pt.y >= 16) {
                continue;
            }

            if ((current_cell.walls & (1 << dir_idx)) != 0) {
                continue;
            }

            const auto& next_cell = grid[next_pt.x][next_pt.y];
            if (!next_cell.visited()) {
                continue;
            }

            CompassHeading new_h = get_next_heading(
                static_cast<CompassHeading>(curr.heading),
                curr.last_step,
                next_dir);

            uint8_t next_run = (new_h == static_cast<CompassHeading>(curr.heading))
                ? static_cast<uint8_t>(std::min<size_t>(curr.run_count + 1, MAX_WEIGHT_STEPS - 1))
                : 0;

            uint16_t step_cost = calculate_step_cost(
                static_cast<CompassHeading>(curr.heading),
                new_h,
                curr.run_count,
                next_run,
                turn_90_ms,
                turn_90_diag_ms);

            uint32_t total_cost = curr.cost_ms + step_cost;
            if (total_cost > 0xFFFE) {
                total_cost = 0xFFFE;
            }

            uint8_t new_h_idx = static_cast<uint8_t>(new_h);
            if (total_cost < cost_matrix[next_pt.x][next_pt.y][new_h_idx]) {
                cost_matrix[next_pt.x][next_pt.y][new_h_idx] = static_cast<uint16_t>(total_cost);

                PackedParent p{};
                p.x = curr.x;
                p.y = curr.y;
                p.heading = curr.heading;
                p.step = dir_idx;
                p.valid = 1;
                parent_matrix[next_pt.x][next_pt.y][new_h_idx] = p;

                heap.push({
                    static_cast<uint16_t>(total_cost),
                    static_cast<uint8_t>(next_pt.x),
                    static_cast<uint8_t>(next_pt.y),
                    new_h_idx,
                    next_run,
                    next_dir
                });
            }
        }
    }

    if (!found_goal) {
        return {};
    }

    if (out_estimated_time_s) {
        *out_estimated_time_s = goal_state.cost_ms / 1000.0f;
    }

    // 5. Backtracking from goal to start_pos
    std::vector<Direction> path;
    uint8_t bx = goal_state.x;
    uint8_t by = goal_state.y;
    uint8_t bh = goal_state.heading;

    while (!(bx == start_pos.x && by == start_pos.y)) {
        PackedParent p = parent_matrix[bx][by][bh];
        if (!p.valid) {
            break;
        }
        path.push_back(static_cast<Direction>(p.step));
        bx = p.x;
        by = p.y;
        bh = p.heading;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace algorithm
