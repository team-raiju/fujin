#pragma once

#include <cstdint>

#include "algorithms/pid.hpp"
#include "utils/movement_params.hpp"


namespace services {

class Control {
    public:
        static Control* instance();

        Control(const Control&) = delete;

        void init();
        void reset(GeneralParams general_params);
        void update();

        void start_fan();
        void stop_fan();
        
        void set_target_linear_speed(float speed) { target_linear_speed_m_s = speed; }
        void set_target_angular_speed(float speed) { target_angular_speed_rad_s = speed; }
        void set_wall_pid_enabled(bool enabled) { wall_pid_enabled = enabled; }
        void set_diagonal_pid_enabled(bool enabled) { diagonal_pid_enabled = enabled; }
        void set_motor_control_disabled(bool disabled) {motor_control_disabled = disabled;}
        
        float get_target_linear_speed() { return target_linear_speed_m_s; }
        float get_target_angular_speed() { return target_angular_speed_rad_s; }
        bool get_wall_pid_enabled() { return wall_pid_enabled; }
        bool get_diagonal_pid_enabled() { return diagonal_pid_enabled; }
        int16_t get_pwm_duty_l() { return pwm_duty_l; }
        int16_t get_pwm_duty_r() { return pwm_duty_r; }
        
        float get_integral_vel() { return linear_vel_pid.get_integral(); }
        float get_integral_angular() { return angular_vel_pid.get_integral(); }
        bool is_emergency() {return emergency; }

    private:
        Control() {}

        algorithm::PID linear_vel_pid;
        algorithm::PID angular_vel_pid;
        algorithm::PID walls_pid;
        algorithm::PID diagonal_walls_pid;

        float target_linear_speed_m_s;
        float target_angular_speed_rad_s;
        bool wall_pid_enabled;
        bool diagonal_pid_enabled;
        int16_t pwm_duty_l;
        int16_t pwm_duty_r;
        bool motor_control_disabled = false;
        bool emergency = false;

        GeneralParams params;

};

}
