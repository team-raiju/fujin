#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>
#include <span>

#include "algorithms/flood_fill.hpp"
#include "utils/movement_params.hpp"
#include "utils/types.hpp"

namespace algorithm {

enum class CompassHeading : uint8_t {
    NORTH = 0,
    NORTH_EAST = 1,
    EAST = 2,
    SOUTH_EAST = 3,
    SOUTH = 4,
    SOUTH_WEST = 5,
    WEST = 6,
    NORTH_WEST = 7,
    NONE = 8
};

constexpr bool is_orthogonal(CompassHeading h) {
    return h == CompassHeading::NORTH || h == CompassHeading::EAST ||
           h == CompassHeading::SOUTH || h == CompassHeading::WEST;
}

constexpr bool is_diagonal(CompassHeading h) {
    return h == CompassHeading::NORTH_EAST || h == CompassHeading::SOUTH_EAST ||
           h == CompassHeading::SOUTH_WEST || h == CompassHeading::NORTH_WEST;
}

constexpr CompassHeading to_compass(Direction d) {
    switch (d) {
    case Direction::NORTH: return CompassHeading::NORTH;
    case Direction::WEST:  return CompassHeading::WEST;
    case Direction::SOUTH: return CompassHeading::SOUTH;
    case Direction::EAST:  return CompassHeading::EAST;
    default:               return CompassHeading::NONE;
    }
}

constexpr Direction to_direction(CompassHeading h) {
    switch (h) {
    case CompassHeading::NORTH: return Direction::NORTH;
    case CompassHeading::WEST:  return Direction::WEST;
    case CompassHeading::SOUTH: return Direction::SOUTH;
    case CompassHeading::EAST:  return Direction::EAST;
    default:                    return Direction::STOP;
    }
}

struct CellWeight {
    float time_ms;
    float penalty_ms;
};

struct PackedParent {
    uint16_t x : 4;
    uint16_t y : 4;
    uint16_t heading : 3;
    uint16_t step : 2;
    uint16_t valid : 1;
};

struct QueueItem {
    uint16_t cost_ms;
    uint8_t x : 4;
    uint8_t y : 4;
    uint8_t heading : 4;
    uint8_t run_count : 4;
    Direction last_step;

    bool operator<(const QueueItem& other) const {
        return cost_ms < other.cost_ms;
    }
};

template <typename T, size_t Capacity>
class MinHeap {
private:
    T data[Capacity];
    size_t size_ = 0;

public:
    constexpr MinHeap() = default;

    void clear() { size_ = 0; }
    [[nodiscard]] bool empty() const { return size_ == 0; }
    [[nodiscard]] size_t size() const { return size_; }
    [[nodiscard]] bool full() const { return size_ >= Capacity; }

    bool push(const T& item) {
        if (size_ >= Capacity) return false;
        size_t i = size_++;
        data[i] = item;
        while (i > 0) {
            size_t parent = (i - 1) / 2;
            if (data[i] < data[parent]) {
                T tmp = data[i];
                data[i] = data[parent];
                data[parent] = tmp;
                i = parent;
            } else {
                break;
            }
        }
        return true;
    }

    T pop() {
        T top = data[0];
        data[0] = data[--size_];
        size_t i = 0;
        while (2 * i + 1 < size_) {
            size_t left = 2 * i + 1;
            size_t right = left + 1;
            size_t smallest = (right < size_ && data[right] < data[left]) ? right : left;
            if (data[smallest] < data[i]) {
                T tmp = data[i];
                data[i] = data[smallest];
                data[smallest] = tmp;
                i = smallest;
            } else {
                break;
            }
        }
        return top;
    }
};

class TimeFloodFill {
public:
    static constexpr size_t MAX_WEIGHT_STEPS = 16;

    static void generate_kinematic_weights(
        float cell_dist, float init_speed, float max_speed, float accel, float decel,
        size_t count, CellWeight* weights_out);

    static CompassHeading get_next_heading(
        CompassHeading curr_heading, Direction last_step, Direction next_step);

    static uint16_t calculate_step_cost(
        CompassHeading curr_heading, CompassHeading next_heading,
        uint8_t curr_run_count, uint8_t next_run_count,
        uint16_t turn_90_ms, uint16_t turn_90_diag_ms);

    static std::vector<Direction> find_fastest_path(
        const Grid<16, 16>& grid,
        Point start_pos,
        std::span<const Point> goals,
        const std::array<ForwardParams, MOVEMENT_COUNT>& fwd_params,
        const std::array<TurnParams, MOVEMENT_COUNT>& trn_params,
        float* out_estimated_time_s = nullptr);
};

} // namespace algorithm
