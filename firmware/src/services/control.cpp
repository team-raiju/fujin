#include <cstdio>

#include "bsp/motors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/analog_sensors.hpp"
#include "services/control.hpp"
#include "services/config.hpp"
#include "bsp/imu.hpp"

static constexpr float max_battery_voltage = 12.6;

namespace services {

Control *Control::instance() {
    static Control p;
    return &p;
}

void Control::init(void) {
    reset();
}

void Control::reset(void) {
    linear_vel_pid.reset();
    linear_vel_pid.kp = Config::linear_vel_kp;
    linear_vel_pid.ki = Config::linear_vel_ki;
    linear_vel_pid.kd = Config::linear_vel_kd;
    linear_vel_pid.integral_limit = 100;

    angular_vel_pid.reset();
    angular_vel_pid.kp = Config::angular_kp;
    angular_vel_pid.ki = Config::angular_ki;
    angular_vel_pid.kd = Config::angular_kd;
    angular_vel_pid.integral_limit = 1000; // approximately (max_bat_voltage / Config::angular_ki)

    walls_pid.reset();
    walls_pid.kp = Config::wall_kp;
    walls_pid.ki = Config::wall_ki;
    walls_pid.kd = Config::wall_kd;
    walls_pid.integral_limit = 0;

    target_angular_speed_rad_s = 0;
    target_linear_speed_m_s = 0;

}

void Control::update() {

    if (wall_pid_enabled){
        target_angular_speed_rad_s += walls_pid.calculate(0.0, bsp::analog_sensors::ir_side_wall_error());
    }

    if (diagonal_pid_enabled) {
        target_angular_speed_rad_s += walls_pid.calculate(0.0, bsp::analog_sensors::ir_diagonal_error());
    }
    
    float mean_velocity_m_s = bsp::encoders::get_filtered_velocity_m_s();

    float linear_ratio = linear_vel_pid.calculate(target_linear_speed_m_s, mean_velocity_m_s);
    float rotation_ratio = -angular_vel_pid.calculate(target_angular_speed_rad_s, bsp::imu::get_rad_per_s());

    float l_voltage = linear_ratio + rotation_ratio;
    float r_voltage = linear_ratio - rotation_ratio;

    pwm_duty_l = (l_voltage / max_battery_voltage) * 1000;
    pwm_duty_r = (r_voltage / max_battery_voltage) * 1000;

    bsp::motors::set(pwm_duty_l, pwm_duty_r);

}

}
