#include "types.hpp"
#include <cstdint>
#include <map>

static constexpr float CELL_SIZE_CM = 18.0;
static constexpr float HALF_CELL_SIZE_CM = 9.0;
static constexpr float CELL_DIAGONAL_SIZE_CM = 12.728;
static constexpr float ROBOT_DIST_FROM_CENTER_START_CM = 2.0;

/**
 * @struct TurnParams
 * @brief Parameters for defining a turn movement.
 *
 *
 * @param start How much the robot will advance before starting the turn in [mm]
 * @param end How much the robot will advance after finishing the turn in [mm]
 * @param turn_linear_speed The linear speed during the turn in [m/s]
 * @param angular_accel The angular acceleration during the turn in [rad/s^2]
 * @param max_angular_speed The maximum angular speed during the turn in [rad/s]
 * @param t_accel The time duration for acceleration and deceleration in [ms]
 * @param t_max_ang_vel The time duration that the robot will be in maximum angular speed in [ms]
 * @param sign The direction sign of the turn (positive or negative).
 *
 */
struct TurnParams {
    float start;
    float end;
    float turn_linear_speed;
    float angular_accel;
    float max_angular_speed;
    uint16_t t_accel;
    uint16_t t_max_ang_vel;
    int sign;

    constexpr TurnParams()
        : start(0), end(0), turn_linear_speed(0), angular_accel(0), max_angular_speed(0), t_accel(0), t_max_ang_vel(0),
          sign(0) {}

    constexpr TurnParams(float s, float e, float ls, float aa, float mas, int ta, int tm, int sg) noexcept
        : start(s), end(e), turn_linear_speed(ls), angular_accel(aa), max_angular_speed(mas), t_accel(ta),
          t_max_ang_vel(tm), sign(sg) {}
};

/**
 * @struct ForwardParams
 * @brief Parameters for defining a forward movement.
 *
 *
 * @param max_speed The maximum speed of the robot in [m/s]
 * @param end_speed The speed of the robot after finishing the movement in [m/s]
 * @param acceleration The acceleration of the robot in [m/s^2]
 * @param deceleration The deceleration of the robot in [m/s^2]
 *
 */
struct ForwardParams {
    float max_speed;
    float end_speed;
    float acceleration;
    float deceleration;
    float target_travel_cm;

    ForwardParams() : max_speed(0), end_speed(0), acceleration(0), deceleration(0), target_travel_cm(0) {}

    ForwardParams(float ms, float es, float a, float d, float t) noexcept
        : max_speed(ms), end_speed(es), acceleration(a), deceleration(d), target_travel_cm(t) {}
};

static std::map<Movement, TurnParams> turn_params_search = {
    {Movement::TURN_RIGHT_90, {33, 33, 0.25, 69.8132, 5.236, 75, 225, -1}},
    {Movement::TURN_LEFT_90, {33, 33, 0.25, 69.8132, 5.236, 75, 225, 1}},
    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 69.8132, 5.236, 60, 540, -1}},
};

static std::map<Movement, ForwardParams> forward_params_search = {
    {Movement::START, {0.25, 0.25, 1.0, 1.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.25, 0.25, 1.0, 1.0, CELL_SIZE_CM}},
    {Movement::STOP, {0.25, 0.0, 1.0, 1.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.25, 0.0, 1.0, 1.0, HALF_CELL_SIZE_CM}},
};

static std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {0, 78, 0.5, 100.00, 7.854, 79, 21, -1}},
    {Movement::TURN_LEFT_45, {0, 78, 0.5, 100.00, 7.854, 79, 21, 1}},
    {Movement::TURN_RIGHT_90, {10, 10, 0.5, 100.00, 8.7266, 87, 93, -1}},
    {Movement::TURN_LEFT_90, {10, 10, 0.5, 100.00, 8.7266, 87, 93, 1}},
    {Movement::TURN_RIGHT_135, {0, 74, 0.5, 100.00, 7.5049, 75, 239, -1}},
    {Movement::TURN_LEFT_135, {0, 74, 0.5, 100.00, 7.5049, 75, 239, 1}},
    {Movement::TURN_RIGHT_180, {0, 0, 0.5, 100.00, 5.5850, 56, 507, -1}},
    {Movement::TURN_LEFT_180, {0, 0, 0.5, 100.00, 5.5850, 56, 507, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {80, 0, 0.5, 100.0, 7.8539, 79, 21, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {80, 0, 0.5, 100.0, 7.8539, 79, 21, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {50, 50, 0.5, 100.0, 10.472, 105, 45, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {50, 50, 0.5, 100.0, 10.472, 105, 45, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {70, 0, 0.5, 100.0, 7.8539, 79, 230, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {70, 0, 0.5, 100.0, 7.8539, 79, 230, 1}},
    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 70, 380, -1}},
};

static std::map<Movement, ForwardParams> forward_params_slow = {
    {Movement::START, {0.5, 0.5, 2.0, 2.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.5, 0.5, 2.0, 2.0, CELL_SIZE_CM}},
    {Movement::FORWARD_BEFORE_TURN_45, {0.5, 0.5, 2.0, 2.0, (CELL_SIZE_CM - 5)}},
    {Movement::DIAGONAL, {0.5, 0.5, 2.0, 2.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::FORWARD_AFTER_DIAGONAL, {0.5, 0.5, 2.0, 2.0, CELL_SIZE_CM - 4.6}},
    {Movement::STOP, {0.5, 0.0, 2.0, 2.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.5, 0.0, 2.0, 2.0, HALF_CELL_SIZE_CM}},
};
