#include <cstdio>

#include "bsp/motors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/analog_sensors.hpp"
#include "services/control.hpp"
#include "services/config.hpp"
#include "bsp/imu.hpp"



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
    linear_vel_pid.integral_limit = 1000;

    angular_vel_pid.reset();
    angular_vel_pid.kp = Config::angular_kp;
    angular_vel_pid.ki = Config::angular_ki;
    angular_vel_pid.kd = Config::angular_kd;
    angular_vel_pid.integral_limit = 1000;

    walls_pid.reset();
    walls_pid.kp = Config::wall_kp;
    walls_pid.ki = Config::wall_ki;
    walls_pid.kd = Config::wall_kd;
    walls_pid.integral_limit = 0;

}

void Control::update() {

    float linear_vel_pwm = linear_vel_pid.calculate(target_linear_speed_m_s, bsp::encoders::get_linear_velocity_m_s());
    float rotation_pwm = -angular_vel_pid.calculate(target_angular_speed_rad_s, bsp::imu::get_rad_per_s());
    if (wall_pid_enabled){
        rotation_pwm += walls_pid.calculate(0.0, bsp::analog_sensors::ir_side_wall_error());
    }

    float speed_l = linear_vel_pwm + rotation_pwm;
    float speed_r = linear_vel_pwm - rotation_pwm;
    bsp::motors::set(speed_l, speed_r);

}

}
