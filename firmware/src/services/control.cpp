#include <cstdio>

#include "bsp/analog_sensors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "services/control.hpp"

static constexpr float max_battery_voltage = 12.6;
static constexpr float mot_kt = 0.0064; // motor torque constant [Nm/A]
static constexpr float mot_ra = 2.5;    // armature resistance[Ohms]

namespace services {

Control* Control::instance() {
    static Control p;
    return &p;
}

void Control::init(void) {
    GeneralParams general_params = {
        services::Config::fan_speed,
        services::Config::angular_kp,
        services::Config::angular_ki,
        services::Config::angular_kd,
        services::Config::wall_kp,
        services::Config::wall_ki,
        services::Config::wall_kd,
        services::Config::linear_vel_kp,
        services::Config::linear_vel_ki,
        services::Config::linear_vel_kd,
        services::Config::diagonal_walls_kp,
        services::Config::diagonal_walls_ki,
        services::Config::diagonal_walls_kd,
        services::Config::start_wall_break_cm_left,
        services::Config::start_wall_break_cm_right,
        services::Config::enable_wall_break_correction,
    };
    reset(general_params);
}

void Control::reset(GeneralParams general_params) {
    params = general_params;

    linear_vel_pid.reset();
    linear_vel_pid.kp = params.linear_vel_kp;
    linear_vel_pid.ki = params.linear_vel_ki;
    linear_vel_pid.kd = params.linear_vel_kd;
    linear_vel_pid.integral_limit = 100;

    angular_vel_pid.reset();
    angular_vel_pid.kp = params.angular_kp;
    angular_vel_pid.ki = params.angular_ki;
    angular_vel_pid.kd = params.angular_kd;
    angular_vel_pid.integral_limit = 500; // approximately (max_bat_voltage / Config::angular_ki)

    walls_pid.reset();
    walls_pid.kp = params.wall_kp;
    walls_pid.ki = params.wall_ki;
    walls_pid.kd = params.wall_kd;
    walls_pid.integral_limit = 0;

    diagonal_walls_pid.reset();
    diagonal_walls_pid.kp = params.diagonal_walls_kp;
    diagonal_walls_pid.ki = params.diagonal_walls_ki;
    diagonal_walls_pid.kd = params.diagonal_walls_kd;
    diagonal_walls_pid.integral_limit = 0;

    target_angular_speed_rad_s = 0;
    target_linear_speed_m_s = 0;

    motor_control_disabled = false;
    emergency = false;
}

void Control::update() {

    float bat_volts = bsp::analog_sensors::battery_latest_reading_mv() / 1000.0;

    if (motor_control_disabled) {
        bsp::motors::set(0, 0);
    } else {
        if (wall_pid_enabled) {
            target_angular_speed_rad_s += walls_pid.calculate(0.0, bsp::analog_sensors::ir_side_wall_error());
        }

        if (diagonal_pid_enabled) {
            target_angular_speed_rad_s += diagonal_walls_pid.calculate(0.0, bsp::analog_sensors::ir_diagonal_error());
        }

        float mean_velocity_m_s = bsp::encoders::get_filtered_velocity_m_s();

        float linear_ratio = linear_vel_pid.calculate(target_linear_speed_m_s, mean_velocity_m_s);
        float rotation_ratio = -angular_vel_pid.calculate(target_angular_speed_rad_s, bsp::imu::get_rad_per_s());

        auto linear_speed_error = std::abs(target_linear_speed_m_s - mean_velocity_m_s);
        auto angular_speed_error = std::abs(target_angular_speed_rad_s - bsp::imu::get_rad_per_s());
        emergency = ((linear_speed_error > 0.75) | (angular_speed_error > 10.0));

        float l_current = linear_ratio + rotation_ratio;
        float r_current = linear_ratio - rotation_ratio;

        float left_ang_vel = bsp::encoders::get_left_filtered_ang_vel_rad_s();
        float right_ang_vel = bsp::encoders::get_right_filtered_ang_vel_rad_s();

        pwm_duty_l = ((l_current * mot_ra + left_ang_vel * mot_kt) / bat_volts) * 1000;
        pwm_duty_r = ((r_current * mot_ra + right_ang_vel * mot_kt) / bat_volts) * 1000;

        /* Limit PWM */
        if (pwm_duty_l > bsp::motors::COUNTER_PERIOD_MAX || pwm_duty_r > bsp::motors::COUNTER_PERIOD_MAX) {
            uint32_t abs_diff = std::abs(pwm_duty_l - pwm_duty_r);
            abs_diff = std::min(abs_diff, static_cast<uint32_t>(2 * bsp::motors::COUNTER_PERIOD_MAX));
            if (pwm_duty_l > pwm_duty_r) {
                pwm_duty_l = bsp::motors::COUNTER_PERIOD_MAX;
                pwm_duty_r = bsp::motors::COUNTER_PERIOD_MAX - abs_diff;
            } else {
                pwm_duty_l = bsp::motors::COUNTER_PERIOD_MAX - abs_diff;
                pwm_duty_r = bsp::motors::COUNTER_PERIOD_MAX;
            }
        }

        if (pwm_duty_l < -bsp::motors::COUNTER_PERIOD_MAX || pwm_duty_r < -bsp::motors::COUNTER_PERIOD_MAX) {
            uint32_t abs_diff = std::abs(pwm_duty_l - pwm_duty_r);
            abs_diff = std::min(abs_diff, static_cast<uint32_t>(2 * bsp::motors::COUNTER_PERIOD_MAX));

            if (pwm_duty_l < pwm_duty_r) {
                pwm_duty_l = -bsp::motors::COUNTER_PERIOD_MAX;
                pwm_duty_r = -bsp::motors::COUNTER_PERIOD_MAX + abs_diff;
            } else {
                pwm_duty_l = -bsp::motors::COUNTER_PERIOD_MAX + abs_diff;
                pwm_duty_r = -bsp::motors::COUNTER_PERIOD_MAX;
            }
        }

        bsp::motors::set(pwm_duty_l, pwm_duty_r);
    }

    /* Fan control */
    float target_fan_speed = std::min(params.fan_speed, 1000.0f);
    float desired_fan_voltage = (target_fan_speed / 1000.0f) * bsp::fan::get_max_fan_voltage();

    if (bat_volts > 5.0) { // To avoid turning on fan when batery measurement is not reliable
        uint16_t fan_pwm = (desired_fan_voltage / bat_volts) * 1000;
        bsp::fan::set(fan_pwm);
    } else {
        bsp::fan::set(0);
    }
}

void Control::start_fan() {
    float target_fan_speed = std::min(params.fan_speed, static_cast<float>(bsp::fan::MAX_SPEED));
    float desired_fan_voltage =
        (target_fan_speed / static_cast<float>(bsp::fan::MAX_SPEED)) * bsp::fan::get_max_fan_voltage();

    for (float volts = 0; volts < desired_fan_voltage; volts += 0.1) {
        float bat_volts = bsp::analog_sensors::battery_latest_reading_mv() / 1000.0;
        if (bat_volts < 5.0) {
            bsp::fan::set(0);
            break;
        }
        uint16_t fan_pwm = (volts / bat_volts) * bsp::fan::MAX_SPEED;
        std::printf("volts: %f, bat: %f, pwm: %d\r\n", volts, bat_volts, fan_pwm);
        bsp::fan::set(fan_pwm);
        bsp::delay_ms(20);
    }
}

void Control::stop_fan() {
    float target_fan_speed = std::min(params.fan_speed, static_cast<float>(bsp::fan::MAX_SPEED));
    float desired_fan_voltage =
        (target_fan_speed / static_cast<float>(bsp::fan::MAX_SPEED)) * bsp::fan::get_max_fan_voltage();
    for (float volts = desired_fan_voltage; volts > 0; volts -= 0.1) {
        float bat_volts = bsp::analog_sensors::battery_latest_reading_mv() / 1000.0;
        if (bat_volts < 5.0) {
            bsp::fan::set(0);
            break;
        }
        uint16_t fan_pwm = (volts / bat_volts) * bsp::fan::MAX_SPEED;
        bsp::fan::set(fan_pwm);
        bsp::delay_ms(20);
    }
    bsp::fan::set(0);
}

}
