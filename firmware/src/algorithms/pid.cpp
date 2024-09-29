#include "algorithms/pid.hpp"
#include "utils/math.hpp"

namespace algorithm {

PID::PID(float kp, float ki, float kd, float integral_limit)
    : kp(kp), ki(ki), kd(kd), integral_limit(integral_limit), integral(0), previous_error(0) {}

float PID::calculate(float const& target, float const& measured_value) {
    float error = target - measured_value;

    integral += error;

    integral = constrain(integral, -integral_limit, integral_limit);

    float derivative = error - previous_error;
    previous_error = error;

    return kp * error + ki * integral + kd * derivative;
}

void PID::reset() {
    integral = 0;
    previous_error = 0;
}

}
