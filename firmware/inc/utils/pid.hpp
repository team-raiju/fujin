#pragma once

template <typename T>
class PID {
public:
    PID(T kp, T ki, T kd, T integral_limit)
        : kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), integral_(0), previous_error_(0) {}
    
    void update_parameters(T kp, T ki, T kd, T integral_limit) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        integral_limit_ = integral_limit;
    }

    T calculate(T target, T measured_value) {
        T error = target - measured_value;

        integral_ += error;
        if (integral_ > integral_limit_) {
            integral_ = integral_limit_;
        } else if (integral_ < -integral_limit_) {
            integral_ = -integral_limit_;
        }

        T derivative = error - previous_error_;
        previous_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        integral_ = 0;
        previous_error_ = 0;
    }

private:
    T kp_;
    T ki_;
    T kd_;
    T integral_limit_;
    T integral_;
    T previous_error_;
};
