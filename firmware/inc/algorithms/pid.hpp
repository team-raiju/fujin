#pragma once

namespace algorithm {

class PID {
public:
    PID() = default;
    PID(float kp, float ki, float kd, float integral_limit, float derivative_filter);

    // Freely updatable constants
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_limit = 0.0f;
    float derivative_filter = 0.0f;

    float calculate(float const& target, float const& measured_value);
    float get_integral() const { return integral; }

    void reset();

    float integral = 0.0f;
    float previous_error = 0.0f;
    float filtered_derivative = 0.0f;
};

}
