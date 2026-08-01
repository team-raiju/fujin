#include "algorithms/pid.hpp"
#include "utils/math.hpp"

namespace algorithm {

PID::PID(float kp, float ki, float kd, float integral_limit, float derivative_filter)
    : kp(kp),
      ki(ki),
      kd(kd),
      integral_limit(integral_limit),
      derivative_filter(derivative_filter),
      integral(0),
      previous_error(0),
      filtered_derivative(0) {}

float PID::calculate(float const& target, float const& measured_value) {
    float error = target - measured_value;

    integral += error;

    integral = constrain(integral, -integral_limit, integral_limit);

    float raw_derivative = error - previous_error;

    filtered_derivative = derivative_filter * raw_derivative + (1.0f - derivative_filter) * filtered_derivative;

    previous_error = error;

    return kp * error + ki * integral + kd * filtered_derivative;
}

void PID::reset() {
    integral = 0;
    previous_error = 0;
    filtered_derivative = 0;
}

}
