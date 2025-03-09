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
 * @param start Where the movement begins in relation to the start of the cell [cm] (can be negative or positive)
 * @param end Where the movement ends in relation to the start of the cell [cm] (can be negative or positive)
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
    {Movement::TURN_RIGHT_45, {-6.1, -7.8, 0.5, 100.00, 7.854, 79, 21, -1}}, //OK
    {Movement::TURN_LEFT_45, {-6.1, -7.8, 0.5, 100.00, 7.854, 79, 21, 1}},
    {Movement::TURN_RIGHT_90, {1.0, -1.6, 0.5, 100.00, 8.7266, 87, 93, -1}}, //OK
    {Movement::TURN_LEFT_90, {1.0, -1.6, 0.5, 100.00, 8.7266, 87, 93, 1}},
    {Movement::TURN_RIGHT_135, {0, -7.4, 0.5, 100.00, 7.5049, 75, 239, -1}},
    {Movement::TURN_LEFT_135, {0, -7.4, 0.5, 100.00, 7.5049, 75, 239, 1}},
    {Movement::TURN_RIGHT_180, {0, 0, 0.5, 100.00, 5.5850, 56, 507, -1}},
    {Movement::TURN_LEFT_180, {0, 0, 0.5, 100.00, 5.5850, 56, 507, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {8.0, 4.6, 0.5, 100.0, 7.8539, 79, 21, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {8.0, 4.6, 0.5, 100.0, 7.8539, 79, 21, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {5.0, -5.0, 0.5, 100.0, 10.472, 105, 45, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {5.0, -5.0, 0.5, 100.0, 10.472, 105, 45, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {7.0, 0, 0.5, 100.0, 7.8539, 79, 230, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {7.0, 0, 0.5, 100.0, 7.8539, 79, 230, 1}},
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


static std::map<Movement, TurnParams> turn_params_medium = {
    {Movement::TURN_RIGHT_45, {-6.1, -7.1, 1.0, 279.253, 13.963, 50, 6, -1}},
    {Movement::TURN_LEFT_45, {-6.1, -7.1, 1.0, 279.253, 13.963, 50, 6, 1}},
    {Movement::TURN_RIGHT_90, {0, 0, 1.0, 279.253, 18.5, 66, 19, -1}},
    {Movement::TURN_LEFT_90, {0, 0, 1.0, 279.253, 18.5, 66, 19, 1}},
    {Movement::TURN_RIGHT_135, {0, -7.5, 1.0, 279.253, 16.581, 59, 85, -1}},
    {Movement::TURN_LEFT_135, {0, -7.5, 1.0, 279.253, 16.581, 59, 85, 1}},
    {Movement::TURN_RIGHT_180, {0, 0, 1.0, 279.253, 11.118, 40, 243, -1}},
    {Movement::TURN_LEFT_180, {0, 0, 1.0, 279.253, 11.118, 40, 243, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {7.3, 5.5, 1.0, 279.253, 13.963, 50, 6, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {7.3, 5.5, 1.0, 279.253, 13.963, 50, 6, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {4.0, -3.8, 1.0, 279.253, 119.199, 69, 13, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {4.0, -3.8, 1.0, 279.253, 19.199, 69, 13, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {8.0, 0, 1.0, 279.253, 15.708, 56, 94, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {8.0, 0, 1.0, 279.253, 15.708, 56, 94, 1}},
    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 70, 380, -1}},
};

static std::map<Movement, ForwardParams> forward_params_medium = {
    {Movement::START, {1.5, 1.5, 6.0, 6.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {2.0, 1.0, 6.0, 10.0, CELL_SIZE_CM}},
    {Movement::FORWARD_BEFORE_TURN_45, {1.0, 1.0, 6.0, 6.0, (CELL_SIZE_CM - 6.1)}},
    {Movement::DIAGONAL, {2.0, 1.0, 6.0, 10.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::FORWARD_AFTER_DIAGONAL, {1.0, 1.0, 6.0, 6.0, CELL_SIZE_CM - 5.5}},
    {Movement::STOP, {1.0, 0.0, 1.0, 15.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {1.0, 0.0, 1.0, 15.0, HALF_CELL_SIZE_CM}},
};

static std::map<Movement, TurnParams> turn_params_fast = {
    {Movement::TURN_RIGHT_45, {0, -8.0, 1.5, 785.39, 22.69, 29, 6, -1}},
    {Movement::TURN_LEFT_45, {0, -8.0, 1.5, 785.39, 22.69, 29, 6, 1}},

    {Movement::TURN_RIGHT_90, {-2.8, -1.3, 1.5, 785.39, 24.96, 32, 31, -1}}, //OK
    {Movement::TURN_LEFT_90, {-2.8, -1.3, 1.5, 785.39, 24.96, 32, 31, 1}},

    {Movement::TURN_RIGHT_135, {0, 7.4, 1.5, 785.39, 7.5049, 75, 239, -1}},
    {Movement::TURN_LEFT_135, {0, 7.4, 1.5, 785.39, 7.5049, 75, 239, 1}},

    {Movement::TURN_RIGHT_180, {-2.7, 0.5, 1.5, 610.865, 16.05, 26, 169, -1}}, //OK
    {Movement::TURN_LEFT_180, {-2.7, 0.5, 1.5, 610.865, 16.05, 26, 169, 1}}, 

    {Movement::TURN_RIGHT_45_FROM_45, {7.3, 5.5, 1.5, 279.253, 13.963, 50, 6, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {7.3, 5.5, 1.5, 279.253, 13.963, 50, 6, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {4.0, -3.8, 1.5, 279.253, 119.199, 69, 13, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {4.0, -3.8, 1.5, 279.253, 19.199, 69, 13, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {8.0, 0, 1.5, 279.253, 15.708, 56, 94, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {8.0, 0, 1.5, 279.253, 15.708, 56, 94, 1}},
    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 70, 380, -1}},
};

static std::map<Movement, ForwardParams> forward_params_fast = {
    {Movement::START, {1.5, 1.5, 10.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},

    // {Movement::FORWARD, {6.7, 1.5, 35.0, 35.0, CELL_SIZE_CM}},
    {Movement::FORWARD, {3.0, 1.5, 20.0, 35.0, CELL_SIZE_CM}},

    {Movement::FORWARD_BEFORE_TURN_45, {1.5, 1.5, 20.0, 35.0, (CELL_SIZE_CM - 6.5)}}, // for 45 degrees
    {Movement::DIAGONAL, {0.5, 0.5, 2.0, 2.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::FORWARD_AFTER_DIAGONAL, {0.5, 0.5, 2.0, 2.0, CELL_SIZE_CM - 4.6}},
    {Movement::STOP, {1.0, 0.0, 2.0, 35.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.5, 0.0, 2.0, 2.0, HALF_CELL_SIZE_CM}},
};

