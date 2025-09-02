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
 * @param start Where the movement begins in relation to the start of the cell [cm] (can be negative (before the cell
 * begin) or positive (after the cell begin))
 * @param end Where the movement ends in relation to the start of the cell [cm] (can be negative (before the cell begin)
 * or positive (after the cell begin))
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
    float angle_accel_rad;
    float angle_max_ang_speed_rad;
    int sign;

    constexpr TurnParams()
        : start(0), end(0), turn_linear_speed(0), angular_accel(0), max_angular_speed(0), t_accel(0), t_max_ang_vel(0),
          angle_accel_rad(0), angle_max_ang_speed_rad(0), sign(0) {}

    constexpr TurnParams(float s, float e, float ls, float aa, float mas, int ta, int tm, float aar, float amr,
                         int sg) noexcept
        : start(s), end(e), turn_linear_speed(ls), angular_accel(aa), max_angular_speed(mas), t_accel(ta),
          t_max_ang_vel(tm), angle_accel_rad(aar), angle_max_ang_speed_rad(amr), sign(sg) {}
};

/**
 * @struct ForwardParams
 * @brief Parameters for defining a forward movement.
 *
 *
 * @param max_speed The maximum speed of the robot in [m/s]
 * @param acceleration The acceleration of the robot in [m/s^2]
 * @param deceleration The deceleration of the robot in [m/s^2]
 * @param target_travel_cm The distance the robot should travel in [cm]
 */
struct ForwardParams {
    float max_speed;
    float acceleration;
    float deceleration;
    float target_travel_cm;

    ForwardParams() : max_speed(0), acceleration(0), deceleration(0), target_travel_cm(0) {}

    ForwardParams(float ms, float a, float d, float t) noexcept
        : max_speed(ms), acceleration(a), deceleration(d), target_travel_cm(t) {}
};

static std::map<Movement, TurnParams> turn_params_search = {
    {Movement::TURN_AROUND, {0, 0, 0.0, 52.36, 3.49, 67, 833, 0.116, 3.025, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0, 0, 0.25, 43.633, 4.014, 92, 299, 0.185, 1.37, -1}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0, 0, 0.25, 43.633, 4.014, 92, 299, 0.185, 1.37, 1}},
};

static std::map<Movement, ForwardParams> forward_params_search = {
    {Movement::START, {0.25, 0.65, 0.65, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.25, 0.65, 0.65, CELL_SIZE_CM}},
    {Movement::STOP, {0.25, 0.65, 0.65, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.25, 0.5, 0.5, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.25, 0.65, 0.65, 2.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.25, 0.65, 0.65, 2.0}},
};

static std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {-6.1, -7.8, 0.5, 100.00, 7.854, 79, 21, 0, 0, -1}},
    {Movement::TURN_LEFT_45, {-6.1, -7.8, 0.5, 100.00, 7.854, 79, 21, 0, 0, 1}},
    {Movement::TURN_RIGHT_90, {1.0, -1.6, 0.5, 100.00, 8.7266, 87, 93, 0.381, 1.190, -1}},
    {Movement::TURN_LEFT_90, {1.0, -1.6, 0.5, 100.00, 8.7266, 87, 93, 0.381, 1.190, 1}},

    // Below parameters not tested
    {Movement::TURN_RIGHT_135, {0, -7.4, 0.5, 100.00, 7.5049, 75, 239, 0, 0, -1}},
    {Movement::TURN_LEFT_135, {0, -7.4, 0.5, 100.00, 7.5049, 75, 239, 0, 0, 1}},
    {Movement::TURN_RIGHT_180, {0, 0, 0.5, 100.00, 5.5850, 56, 507, 0, 0, -1}},
    {Movement::TURN_LEFT_180, {0, 0, 0.5, 100.00, 5.5850, 56, 507, 0, 0, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {9.5, 4.6, 0.5, 100.0, 7.8539, 79, 21, 0, 0, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {9.5, 4.6, 0.5, 100.0, 7.8539, 79, 21, 0, 0, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {5.0, -5.0, 0.5, 100.0, 10.472, 105, 45, 0, 0, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {5.0, -5.0, 0.5, 100.0, 10.472, 105, 45, 0, 0, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {7.0, 0, 0.5, 100.0, 7.8539, 79, 230, 0, 0, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {7.0, 0, 0.5, 100.0, 7.8539, 79, 230, 0, 0, 1}},
    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 70, 380, 0, 0, -1}},
};

static std::map<Movement, ForwardParams> forward_params_slow = {
    {Movement::START, {0.5, 2.0, 2.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.5, 2.0, 2.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {0.5, 2.0, 2.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {0.5, 2.0, 2.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.5, 2.0, 2.0, HALF_CELL_SIZE_CM}},
};

static std::map<Movement, TurnParams> turn_params_medium = {
    // Below parameters not tested
    {Movement::TURN_RIGHT_45, {-6.1, -7.1, 1.0, 279.253, 13.963, 50, 6, 0, 0, -1}},
    {Movement::TURN_LEFT_45, {-6.1, -7.1, 1.0, 279.253, 13.963, 50, 6, 0, 0, 1}},
    {Movement::TURN_RIGHT_90, {0, 0, 1.0, 279.253, 18.5, 66, 19, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {0, 0, 1.0, 279.253, 18.5, 66, 19, 0, 0, 1}},
    {Movement::TURN_RIGHT_135, {0, -7.5, 1.0, 279.253, 16.581, 59, 85, 0, 0, -1}},
    {Movement::TURN_LEFT_135, {0, -7.5, 1.0, 279.253, 16.581, 59, 85, 0, 0, 1}},
    {Movement::TURN_RIGHT_180, {0, 0, 1.0, 279.253, 11.118, 40, 243, 0, 0, -1}},
    {Movement::TURN_LEFT_180, {0, 0, 1.0, 279.253, 11.118, 40, 243, 0, 0, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {7.3, 5.5, 1.0, 279.253, 13.963, 50, 6, 0, 0, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {7.3, 5.5, 1.0, 279.253, 13.963, 50, 6, 0, 0, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {4.0, -3.8, 1.0, 279.253, 119.199, 69, 13, 0, 0, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {4.0, -3.8, 1.0, 279.253, 19.199, 69, 13, 0, 0, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {8.0, 0, 1.0, 279.253, 15.708, 56, 94, 0, 0, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {8.0, 0, 1.0, 279.253, 15.708, 56, 94, 0, 0, 1}},
    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 70, 380, 0, 0, -1}},
};

static std::map<Movement, ForwardParams> forward_params_medium = {
    {Movement::START, {1.0, 5.0, 6.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {2.0, 6.0, 10.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {2.0, 6.0, 10.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 1.0, 15.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {1.0, 1.0, 15.0, HALF_CELL_SIZE_CM}},
};

static std::map<Movement, TurnParams> turn_params_fast = {
    {Movement::TURN_RIGHT_45, {-2.0, -8.2, 1.5, 785.39, 22.69, 28, 5, 0, 0, -1}},
    {Movement::TURN_LEFT_45, {-2.0, -8.2, 1.5, 785.39, 22.69, 28, 5, 0, 0, 1}},

    {Movement::TURN_RIGHT_90, {0.0, -1.3, 1.5, 785.39, 24.96, 32, 28, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {0.0, -1.3, 1.5, 785.39, 24.96, 32, 28, 0, 0, 1}},
    {Movement::TURN_RIGHT_135, {-2.9, -9.1, 1.5, 610.865, 20.94, 34, 78, 0, 0, -1}},
    {Movement::TURN_LEFT_135, {-2.9, -9.1, 1.5, 610.865, 20.94, 34, 78, 0, 0, 1}},
    {Movement::TURN_RIGHT_180, {-2.9, 0.2, 1.5, 610.865, 16.05, 26, 169, 0, 0, -1}},
    {Movement::TURN_LEFT_180, {-2.9, 0.2, 1.5, 610.865, 16.05, 26, 169, 0, 0, 1}},

    // Below parameters not tested
    {Movement::TURN_RIGHT_45_FROM_45, {7.7, 5.4, 1.5, 785.39, 22.69, 29, 6, 0, 0, -1}},

    {Movement::TURN_LEFT_45_FROM_45, {7.7, 5.4, 1.5, 785.39, 22.69, 29, 6, 0, 0, 1}},

    {Movement::TURN_RIGHT_90_FROM_45, {4.0, -3.8, 1.5, 785.39, 24.96, 32, 31, 0, 0, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {4.0, -3.8, 1.5, 785.39, 24.96, 32, 31, 0, 0, 1}},

    {Movement::TURN_RIGHT_135_FROM_45, {7.0, 1.0, 1.5, 610.865, 20.94, 34, 78, 0, 0, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {70, 1.0, 1.5, 610.865, 20.94, 34, 78, 0, 0, 1}},

    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 70, 380, 0, 0, -1}},
};

static std::map<Movement, ForwardParams> forward_params_fast = {
    {Movement::START,
     {1.0, 5.0, 20.0,
      HALF_CELL_SIZE_CM +
          ROBOT_DIST_FROM_CENTER_START_CM}}, // Speed reached in the start end is = 1.0m/s (sqrt(2*5*0.11))

    {Movement::FORWARD, {3.5, 15.0, 30.0, CELL_SIZE_CM}},

    {Movement::DIAGONAL, {2.5, 15.0, 35.0, CELL_DIAGONAL_SIZE_CM}},

    {Movement::STOP, {0.75, 2.0, 35.0, (HALF_CELL_SIZE_CM)}},

    {Movement::TURN_AROUND, {0.5, 2.0, 2.0, HALF_CELL_SIZE_CM}},
};
