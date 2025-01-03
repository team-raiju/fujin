#pragma once

#include <cstdint>

#include "algorithms/pid.hpp"


namespace services {

class Control {
    public:
        static Control* instance();

        Control(const Control&) = delete;

        void init();
        void reset();
        void update();

        void set_target_linear_speed(float speed) { target_linear_speed_m_s = speed; }
        void set_target_angular_speed(float speed) { target_angular_speed_rad_s = speed; }
        void set_wall_pid_enabled(bool enabled) { wall_pid_enabled = enabled; }

        float get_target_linear_speed() { return target_linear_speed_m_s; }
        float get_target_angular_speed() { return target_angular_speed_rad_s; }
        bool get_wall_pid_enabled() { return wall_pid_enabled; }
        int16_t get_pwm_duty_l() { return pwm_duty_l; }
        int16_t get_pwm_duty_r() { return pwm_duty_r; }

    private:
        Control() {}

        algorithm::PID linear_vel_pid;
        algorithm::PID angular_vel_pid;
        algorithm::PID walls_pid;

        float target_linear_speed_m_s;
        float target_angular_speed_rad_s;
        bool wall_pid_enabled;
        int16_t pwm_duty_l;
        int16_t pwm_duty_r;

};

}
