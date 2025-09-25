#pragma once
#include "types.hpp"
#include <cstdint>
#include <map>

static constexpr float CELL_SIZE_CM = 18.0;
static constexpr float HALF_CELL_SIZE_CM = 9.0;
static constexpr float CELL_DIAGONAL_SIZE_CM = 12.727922;
static constexpr float ROBOT_DIST_FROM_CENTER_START_CM_FAST = 2.50; // To account for slippery when fast mode
static constexpr float ROBOT_DIST_FROM_CENTER_START_CM = 1.55;

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
 * @param angle_to_turn The angle to turn in [rad]
 * @param t_start_deccel The time elapsed when deceleration is going to start [ms]
 * @param t_stop The time elapsed when curve is going to stop [ms]
 * @param sign The direction sign of the turn (positive = LEFT or negative = RIGHT).
 *
 */
struct TurnParams {
    float start;
    float end;
    float turn_linear_speed;
    float angular_accel;
    float max_angular_speed;
    float angle_to_turn;
    uint16_t t_start_deccel;
    uint16_t t_stop;
    int sign;

    constexpr TurnParams()
        : start(0), end(0), turn_linear_speed(0), angular_accel(0), max_angular_speed(0), angle_to_turn(0),
          t_start_deccel(0), t_stop(0), sign(0) {}

    constexpr TurnParams(float s, float e, float ls, float aa, float mas, float att, float tsd, float ts,
                         int sg) noexcept
        : start(s), end(e), turn_linear_speed(ls), angular_accel(aa), max_angular_speed(mas), angle_to_turn(att),
          t_start_deccel(tsd), t_stop(ts), sign(sg) {}
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

struct GeneralParams {
    float fan_speed;

    float angular_kp;
    float angular_ki;
    float angular_kd;

    float wall_kp;
    float wall_ki;
    float wall_kd;

    float linear_vel_kp;
    float linear_vel_ki;
    float linear_vel_kd;

    float diagonal_walls_kp;
    float diagonal_walls_ki;
    float diagonal_walls_kd;

    float start_wall_break_cm_left;
    float start_wall_break_cm_right;
    float enable_wall_break_correction;

    GeneralParams() : fan_speed(0), angular_kp(0), angular_ki(0), angular_kd(0), wall_kp(0), wall_ki(0), wall_kd(0),
                      linear_vel_kp(0), linear_vel_ki(0), linear_vel_kd(0), diagonal_walls_kp(0), diagonal_walls_ki(0),
                      diagonal_walls_kd(0), start_wall_break_cm_left(0), start_wall_break_cm_right(0),
                      enable_wall_break_correction(0) {}

    GeneralParams(float fan, float akp, float aki, float akd, float wkp, float wki, float wkd, float lvkp, float lvki,
                  float lvkd, float dwkp, float dwki, float dwkd, float swbcl, float swbcr, float ewbc)
        : fan_speed(fan), angular_kp(akp), angular_ki(aki), angular_kd(akd), wall_kp(wkp), wall_ki(wki), wall_kd(wkd),
          linear_vel_kp(lvkp), linear_vel_ki(lvki), linear_vel_kd(lvkd), diagonal_walls_kp(dwkp),
          diagonal_walls_ki(dwki), diagonal_walls_kd(dwkd), start_wall_break_cm_left(swbcl),
          start_wall_break_cm_right(swbcr), enable_wall_break_correction(ewbc) {}
};

extern std::map<Movement, TurnParams> turn_params_search_slow ;
extern std::map<Movement, ForwardParams> forward_params_search_slow ;
extern std::map<Movement, TurnParams> turn_params_search_medium ;
extern std::map<Movement, ForwardParams> forward_params_search_medium ;
extern std::map<Movement, TurnParams> turn_params_search ;
extern std::map<Movement, ForwardParams> forward_params_search ;
extern std::map<Movement, TurnParams> turn_params_slow ;
extern std::map<Movement, ForwardParams> forward_params_slow ;
extern std::map<Movement, TurnParams> turn_params_medium ;
extern std::map<Movement, ForwardParams> forward_params_medium ;
extern std::map<Movement, TurnParams> turn_params_fast ;
extern std::map<Movement, ForwardParams> forward_params_fast ;
extern std::map<Movement, TurnParams> turn_params_custom ;
extern std::map<Movement, ForwardParams> forward_params_custom ;
extern GeneralParams general_params_search_slow ;
extern GeneralParams general_params_search_medium ;
extern GeneralParams general_params_search_fast ;
extern GeneralParams general_params_slow ;
extern GeneralParams general_params_medium ;
extern GeneralParams general_params_fast ;
