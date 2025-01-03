#include <map>
#include <cstdint>
#include "types.hpp"

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

    constexpr TurnParams(): start(0), end(0), turn_linear_speed(0), angular_accel(0),
                           max_angular_speed(0), t_accel(0), t_max_ang_vel(0), sign(0) {}

    constexpr TurnParams(float s, float e, float ls, float aa, 
                        float mas, int ta, int tm, int sg) noexcept
        : start(s), end(e), turn_linear_speed(ls), angular_accel(aa),
          max_angular_speed(mas), t_accel(ta), t_max_ang_vel(tm), sign(sg) {}
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

    constexpr ForwardParams(): max_speed(0), end_speed(0), acceleration(0), deceleration(0) {}

    constexpr ForwardParams(float ms, float es, float a, float d) noexcept
        : max_speed(ms), end_speed(es), acceleration(a), deceleration(d) {}
};


static std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {0, 106, 0.5, 610.87, 16.8249, 28, 19, -1}},
    {Movement::TURN_LEFT_45, {0, 106, 0.5, 610.87, 16.8249, 28, 19, 1}},
    {Movement::TURN_RIGHT_90, {35, 35, 0.5, 610.87, 9.77, 16, 145, -1}},
    {Movement::TURN_LEFT_90, {35, 35, 0.5, 610.87, 9.77, 16, 145, 1}},
    {Movement::TURN_RIGHT_135, {0, 75, 0.5, 610.87, 6.7195, 11, 340, -1}},
    {Movement::TURN_LEFT_135, {0, 75, 0.5, 610.87, 6.7195, 11, 340, 1}},
    {Movement::TURN_RIGHT_180, {0, 0, 0.5, 349.065, 5.5850, 16, 564, -1}},
    {Movement::TURN_LEFT_180, {0, 0, 0.5, 349.065, 5.5850, 16, 564, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {82, 0, 0.5, 610.87, 20.857, 34, 4, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {82, 0, 0.5, 610.87, 20.857, 34, 4, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {77, 0, 0.5, 610.87, 6.7195, 11, 340, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {77, 0, 0.5, 610.87, 6.7195, 11, 340, 1}},
    {Movement::TURN_AROUND, {0, 180, 0.1, 349.065, 5.5850, 16, 564, -1}},
};

static std::map<Movement, ForwardParams> forward_params_slow = {
    {Movement::FORWARD, {0.5, 0.5, 2.0, 2.0}},
    {Movement::STOP, {0.5, 0.0, 2.0, 2.0}},
};
