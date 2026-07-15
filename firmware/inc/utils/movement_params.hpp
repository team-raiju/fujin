#pragma once
#include "types.hpp"
#include <cstdint>
#include <map>

static constexpr float CELL_SIZE_MM = 180.0;
static constexpr float HALF_CELL_SIZE_MM = 90.0;
static constexpr float CELL_DIAGONAL_SIZE_MM = 127.27922;
static constexpr float ROBOT_DIST_FROM_CENTER_START_MM_FAST = 21.0; // To account for slippery when fast mode
static constexpr float ROBOT_DIST_FROM_CENTER_START_MM = 19.0;  // Actually 15.5, But also account for slippery

/**
 * @struct TurnParams
 * @brief Parameters for defining a turn movement.
 *
 *
 * @param start Where the movement begins in relation to the start of the cell [mm] (can be negative (before the cell
 * begin) or positive (after the cell begin))
 * @param end Where the movement ends in relation to the start of the cell [mm] (can be negative (before the cell begin)
 * or positive (after the cell begin))
 * @param turn_linear_speed The linear speed during the turn in [m/s]
 * @param angular_accel The angular acceleration during the turn in [rad/s^2]
 * @param max_angular_speed The maximum angular speed during the turn in [rad/s]
 * @param t_start_deccel The time elapsed when deceleration is going to start [ms]
 * @param t_stop The time elapsed when curve is going to stop [ms]
 * @param sign The direction sign of the turn (positive = LEFT or negative = RIGHT).
 * @param time_to_decrease_jerk_1 Time to decrease jerk 1 in [ms]
 * @param time_to_decrease_jerk_2 Time to decrease jerk 2 in [ms]
 * @param hardcoded_jerk Jerk limit value
 *
 */
struct TurnParams {
    float start;
    float end;
    float turn_linear_speed;
    float angular_accel;
    float max_angular_speed;
    uint16_t t_start_deccel;
    uint16_t t_stop;
    int sign;
    uint16_t time_to_decrease_jerk_1;
    uint16_t time_to_decrease_jerk_2;
    float jerk;

    constexpr TurnParams()
        : start(0), end(0), turn_linear_speed(0), angular_accel(0), max_angular_speed(0),
          t_start_deccel(0), t_stop(0), sign(0), time_to_decrease_jerk_1(0), time_to_decrease_jerk_2(0),
          jerk(0) {}

    constexpr TurnParams(float s, float e, float ls, float aa, float mas, uint16_t tsd, uint16_t ts,
                         int sg, uint16_t j1 = 0, uint16_t j2 = 0, float j = 0.0f) noexcept
        : start(s), end(e), turn_linear_speed(ls), angular_accel(aa), max_angular_speed(mas),
          t_start_deccel(tsd), t_stop(ts), sign(sg), time_to_decrease_jerk_1(j1), time_to_decrease_jerk_2(j2),
          jerk(j) {}
};

/**
 * @struct ForwardParams
 * @brief Parameters for defining a forward movement.
 *
 *
 * @param max_speed The maximum speed of the robot in [m/s]
 * @param acceleration The acceleration of the robot in [m/s^2]
 * @param deceleration The deceleration of the robot in [m/s^2]
 * @param target_travel_mm The distance the robot should travel in [mm]
 */
struct ForwardParams {
    float max_speed;
    float acceleration;
    float deceleration;
    float target_travel_mm;

    ForwardParams() : max_speed(0), acceleration(0), deceleration(0), target_travel_mm(0) {}

    ForwardParams(float ms, float a, float d, float t) noexcept
        : max_speed(ms), acceleration(a), deceleration(d), target_travel_mm(t) {}
};

struct GeneralParams {
    float fan_speed;

    float angular_kp;
    float angular_ki;
    float angular_kd;
    float angular_acc_feed_forward_k;
    float angular_vel_feed_forward_k;

    float linear_vel_acc_feed_forward_k;
    float linear_vel_brake_feed_forward_k;
    float linear_vel_feed_forward_k;
    float linear_jerk_ff_k;
    float linear_jerk_ff_ms;
    float angular_jerk_ff_k;
    float angular_jerk_ff_ms;

    float wall_kp;
    float wall_ki;
    float wall_kd;

    float linear_vel_kp;
    float linear_vel_ki;
    float linear_vel_kd;

    float diagonal_walls_kp;
    float diagonal_walls_ki;
    float diagonal_walls_kd;

    float start_wall_break_mm_left;
    float start_wall_break_mm_right;
    float enable_wall_break_correction;

    GeneralParams()
        : fan_speed(0), angular_kp(0), angular_ki(0), angular_kd(0), angular_acc_feed_forward_k(0),
          angular_vel_feed_forward_k(0), linear_vel_acc_feed_forward_k(0), linear_vel_brake_feed_forward_k(0),
          linear_vel_feed_forward_k(0), linear_jerk_ff_k(0), linear_jerk_ff_ms(0), angular_jerk_ff_k(0), angular_jerk_ff_ms(0),
          wall_kp(0), wall_ki(0), wall_kd(0),
          linear_vel_kp(0), linear_vel_ki(0), linear_vel_kd(0), diagonal_walls_kp(0), diagonal_walls_ki(0),
          diagonal_walls_kd(0), start_wall_break_mm_left(0), start_wall_break_mm_right(0),
          enable_wall_break_correction(0) {}

    GeneralParams(float fan, float akp, float aki, float akd, float aaff, float avff, float lvaff, float lvbff, float lvff, float ljffk, float ljffms, float ajffk, float ajffms, float wkp, float wki,
                  float wkd, float lvkp, float lvki, float lvkd, float dwkp, float dwki, float dwkd, float swbcl,
                  float swbcr, float ewbc)
        : fan_speed(fan), angular_kp(akp), angular_ki(aki), angular_kd(akd), angular_acc_feed_forward_k(aaff),
          angular_vel_feed_forward_k(avff), linear_vel_acc_feed_forward_k(lvaff), linear_vel_brake_feed_forward_k(lvbff),
          linear_vel_feed_forward_k(lvff), linear_jerk_ff_k(ljffk), linear_jerk_ff_ms(ljffms), angular_jerk_ff_k(ajffk), angular_jerk_ff_ms(ajffms),
          wall_kp(wkp), wall_ki(wki),
          wall_kd(wkd), linear_vel_kp(lvkp), linear_vel_ki(lvki), linear_vel_kd(lvkd), diagonal_walls_kp(dwkp),
          diagonal_walls_ki(dwki), diagonal_walls_kd(dwkd), start_wall_break_mm_left(swbcl),
          start_wall_break_mm_right(swbcr), enable_wall_break_correction(ewbc) {}
};

extern const std::map<Movement, TurnParams> turn_params_search_slow;
extern const std::map<Movement, ForwardParams> forward_params_search_slow;
extern const std::map<Movement, TurnParams> turn_params_search_medium;
extern const std::map<Movement, ForwardParams> forward_params_search_medium;
extern const std::map<Movement, TurnParams> turn_params_search_fast;
extern const std::map<Movement, ForwardParams> forward_params_search_fast;
extern const std::map<Movement, TurnParams> turn_params_slow;
extern const std::map<Movement, ForwardParams> forward_params_slow;
extern const std::map<Movement, TurnParams> turn_params_medium;
extern const std::map<Movement, ForwardParams> forward_params_medium;
extern const std::map<Movement, TurnParams> turn_params_fast;
extern const std::map<Movement, ForwardParams> forward_params_fast;
extern const std::map<Movement, TurnParams> turn_params_super;
extern const std::map<Movement, ForwardParams> forward_params_super;
extern std::map<Movement, TurnParams> turn_params_custom;
extern std::map<Movement, ForwardParams> forward_params_custom;
extern const GeneralParams general_params_search_slow;
extern const GeneralParams general_params_search_medium;
extern const GeneralParams general_params_search_fast;
extern const GeneralParams general_params_slow;
extern const GeneralParams general_params_medium;
extern const GeneralParams general_params_fast;
extern const GeneralParams general_params_super;
