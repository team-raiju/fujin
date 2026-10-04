#pragma once

#include <cstdint>
#include <vector>
#include <utility>
#include <string>

#include "utils/types.hpp"
#include "utils/movement_params.hpp"

namespace algorithm {

enum class MovementMode {
    NORMAL = 0,
    SMOOTH = 1,
    DIAGONALS = 2,
    TIME_BASED = 3,
    HARD_CODED = 4
};

enum class PathState {
    Start,
    Ortho_F,  // Moving straight
    Ortho_R,  // Made a single Right 90 turn
    Ortho_L,  // Made a single Left 90 turn
    Ortho_RR, // Made two Right 90 turns (180)
    Ortho_LL, // Made two Left 90 turns (180)
    Diag_LR,  // On a diagonal path, last turn was Right
    Diag_RL,  // On a diagonal path, last turn was Left
    Diag_RR,  // In a diagonal turn sequence (R-R)
    Diag_LL,  // In a diagonal turn sequence (L-L)
    Stop,
};

class MovementPlanner {
public:
    static Movement get_movement(Direction target_dir, Direction current_dir, bool search_mode = false);

    static std::vector<std::pair<Movement, uint8_t>>
    get_default_target_movements(const std::vector<Direction>& target_directions);

    static std::vector<std::pair<Movement, uint8_t>>
    get_smooth_movements(const std::vector<std::pair<Movement, uint8_t>>& default_target_movements);

    static std::vector<std::pair<Movement, uint8_t>>
    get_diagonal_movements(const std::vector<std::pair<Movement, uint8_t>>& default_target_movements);

    static std::vector<std::pair<Movement, uint8_t>>
    plan_movements(const std::vector<Direction>& target_directions, MovementMode mode);
};

} // namespace algorithm
