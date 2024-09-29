#pragma once

namespace algorithm {

class PID {
public:
    PID(float kp, float ki, float kd, float integral_limit);

    // Freely updatable constants
    float kp;
    float ki;
    float kd;
    float integral_limit;

    float calculate(float const& target, float const& measured_value);

    void reset();

private:
    float integral;
    float previous_error;
};

}
