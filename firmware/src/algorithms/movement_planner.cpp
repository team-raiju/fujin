#include "algorithms/movement_planner.hpp"

namespace algorithm {

Movement MovementPlanner::get_movement(Direction target_dir, Direction current_dir, bool search_mode) {
    using enum Direction;
    if (target_dir == Direction::STOP)
        return Movement::STOP;
    if (target_dir == current_dir)
        return Movement::FORWARD;
    if ((target_dir == NORTH && current_dir == WEST) || (target_dir == EAST && current_dir == NORTH) ||
        (target_dir == SOUTH && current_dir == EAST) || (target_dir == WEST && current_dir == SOUTH)) {
        return search_mode ? Movement::TURN_RIGHT_90_SEARCH_MODE : Movement::TURN_RIGHT_90;
    }
    if ((target_dir == NORTH && current_dir == SOUTH) || (target_dir == EAST && current_dir == WEST) ||
        (target_dir == SOUTH && current_dir == NORTH) || (target_dir == WEST && current_dir == EAST)) {
        return Movement::TURN_AROUND;
    }
    return search_mode ? Movement::TURN_LEFT_90_SEARCH_MODE : Movement::TURN_LEFT_90;
}

std::vector<std::pair<Movement, uint8_t>>
MovementPlanner::get_default_target_movements(const std::vector<Direction>& target_directions) {
    std::vector<std::pair<Movement, uint8_t>> default_target_movements = {};

    Direction robot_direction = Direction::NORTH;
    default_target_movements.push_back({Movement::START, 1});

    for (auto target_dir : target_directions) {
        Movement movement = get_movement(target_dir, robot_direction, false);
        default_target_movements.push_back({movement, 1});
        robot_direction = target_dir;
    }

    default_target_movements.push_back({Movement::STOP, 1});
    return default_target_movements;
}

std::vector<std::pair<Movement, uint8_t>>
MovementPlanner::get_smooth_movements(const std::vector<std::pair<Movement, uint8_t>>& default_target_movements) {
    std::vector<std::pair<Movement, uint8_t>> smooth_movements = {};

    smooth_movements.push_back(default_target_movements[0]);
    uint8_t forward_count = 1;
    for (uint32_t i = 1; i < default_target_movements.size() - 1; i++) {
        Movement movement = default_target_movements[i].first;
        Movement next_movement = default_target_movements[i + 1].first;
        if (movement == Movement::TURN_LEFT_90 && next_movement == Movement::TURN_LEFT_90) {
            smooth_movements.push_back({Movement::TURN_LEFT_180, 1});
            i++;
        } else if (movement == Movement::TURN_RIGHT_90 && next_movement == Movement::TURN_RIGHT_90) {
            smooth_movements.push_back({Movement::TURN_RIGHT_180, 1});
            i++;
        } else if (movement == Movement::FORWARD && next_movement == Movement::FORWARD) {
            forward_count++;
        } else if (movement == Movement::FORWARD && next_movement != Movement::FORWARD) {
            smooth_movements.push_back({Movement::FORWARD, forward_count});
            forward_count = 1;
        } else {
            smooth_movements.push_back(default_target_movements[i]);
        }
    }
    smooth_movements.push_back({Movement::STOP, 1});
    return smooth_movements;
}

std::vector<std::pair<Movement, uint8_t>>
MovementPlanner::get_diagonal_movements(const std::vector<std::pair<Movement, uint8_t>>& default_target_movements) {
    if (default_target_movements.empty()) {
        return {};
    }

    std::vector<Movement> flat_moves;
    for (const auto& move_pair : default_target_movements) {
        for (uint8_t i = 0; i < move_pair.second; ++i) {
            flat_moves.push_back(move_pair.first);
        }
    }

    std::vector<std::pair<Movement, uint8_t>> output_movements;
    PathState state = PathState::Start;
    uint8_t run_length = 0;

    for (const auto& move : flat_moves) {
        switch (state) {
        case PathState::Start:
            if (move == Movement::START) {
                output_movements.push_back({Movement::START, 1});
                state = PathState::Ortho_F;
                run_length = 0;
            } else if (move == Movement::STOP) {
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_F:
            if (move == Movement::FORWARD) {
                run_length++;
            } else {
                if (run_length > 0) {
                    output_movements.push_back({Movement::FORWARD, run_length});
                    run_length = 0;
                }
                if (move == Movement::TURN_RIGHT_90)
                    state = PathState::Ortho_R;
                else if (move == Movement::TURN_LEFT_90)
                    state = PathState::Ortho_L;
                else if (move == Movement::STOP)
                    state = PathState::Stop;
            }
            break;

        case PathState::Ortho_R:
            if (move == Movement::FORWARD) {
                output_movements.push_back({Movement::TURN_RIGHT_90, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_RIGHT_90) {
                state = PathState::Ortho_RR;
            } else if (move == Movement::TURN_LEFT_90) {
                output_movements.push_back({Movement::TURN_RIGHT_45, 1});
                run_length = 0;
                state = PathState::Diag_RL;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_RIGHT_90, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_L:
            if (move == Movement::FORWARD) {
                output_movements.push_back({Movement::TURN_LEFT_90, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_LEFT_90) {
                state = PathState::Ortho_LL;
            } else if (move == Movement::TURN_RIGHT_90) {
                output_movements.push_back({Movement::TURN_LEFT_45, 1});
                run_length = 0;
                state = PathState::Diag_LR;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_LEFT_90, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_RR:
            if (move == Movement::FORWARD) {
                output_movements.push_back({Movement::TURN_RIGHT_180, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_LEFT_90) {
                output_movements.push_back({Movement::TURN_RIGHT_135, 1});
                run_length = 0;
                state = PathState::Diag_RL;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_RIGHT_180, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_LL:
            if (move == Movement::FORWARD) {
                output_movements.push_back({Movement::TURN_LEFT_180, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_RIGHT_90) {
                output_movements.push_back({Movement::TURN_LEFT_135, 1});
                run_length = 0;
                state = PathState::Diag_LR;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_LEFT_180, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_RL:
            if (move == Movement::FORWARD) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_45_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_RIGHT_90) {
                run_length++;
                state = PathState::Diag_LR;
            } else if (move == Movement::TURN_LEFT_90) {
                state = PathState::Diag_LL;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_45_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_LR:
            if (move == Movement::FORWARD) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_45_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_LEFT_90) {
                run_length++;
                state = PathState::Diag_RL;
            } else if (move == Movement::TURN_RIGHT_90) {
                state = PathState::Diag_RR;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_45_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_LL:
            if (move == Movement::TURN_RIGHT_90) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_90_FROM_45, 1});
                run_length = 0;
                state = PathState::Diag_LR;
            } else if (move == Movement::FORWARD) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_135_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_135_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_RR:
            if (move == Movement::TURN_LEFT_90) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_90_FROM_45, 1});
                run_length = 0;
                state = PathState::Diag_RL;
            } else if (move == Movement::FORWARD) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_135_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_135_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Stop:
            break;
        }
    }

    output_movements.push_back({Movement::STOP, 1});
    return output_movements;
}

std::vector<std::pair<Movement, uint8_t>>
MovementPlanner::plan_movements(const std::vector<Direction>& target_directions, MovementMode mode) {
    auto default_moves = get_default_target_movements(target_directions);
    switch (mode) {
    case MovementMode::NORMAL:
        return default_moves;
    case MovementMode::SMOOTH:
        return get_smooth_movements(default_moves);
    case MovementMode::DIAGONALS:
    case MovementMode::TIME_BASED:
        return get_diagonal_movements(default_moves);
    case MovementMode::HARD_CODED:
    default:
        return default_moves;
    }
}

} // namespace algorithm
