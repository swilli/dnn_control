#include "controllerfullstate.h"

ControllerFullState::ControllerFullState(const double &control_frequency, const double &maximum_thrust, const Vector3D &target_position) : Controller(7, maximum_thrust),
    control_interval_(1.0 / control_frequency) {
    for (unsigned int i = 0; i < 3; ++i) {
        target_position_[i] = target_position[i];
        previous_error_[i] = 0.0;
        integral_[i] = 0.0;
    }

    constant_integral_ = 0.0;
    constant_derivative_ = 5;
    constant_proportional_ = 4;
}

ControllerFullState::~ControllerFullState() {

}

Vector3D ControllerFullState::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        const double error = target_position_[i] - sensor_data[i];
        const double derivative = (error - previous_error_[i]) / control_interval_;
        integral_[i] += error * control_interval_;
        double t = constant_proportional_ * error + constant_derivative_ * derivative + constant_integral_ * integral_[i];
        if (t > maximum_thrust_) {
            t = maximum_thrust_;
        } else if (t < -maximum_thrust_) {
            t = -maximum_thrust_;
        }
        thrust[i] = t;
        previous_error_[i] = error;
    }
    return thrust;
}
