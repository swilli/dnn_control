#include "controllerfullstate.h"

const unsigned int ControllerFullState::kDimensions = 8;

ControllerFullState::ControllerFullState(const double &maximum_thrust, const Vector3D &target_position) : Controller(kDimensions, maximum_thrust) {
    for (unsigned int i = 0; i < 3; ++i) {
        target_position_[i] = target_position[i];
        previous_error_[i] = 0.0;
        integral_[i] = 0.0;
    }

    latest_control_action_ = 0.0;

    coef_integral_ = 4.0;
    coef_derivative_ = 5.0;
    coef_proportional_ = 0.0;
}

ControllerFullState::~ControllerFullState() {

}

Controller *ControllerFullState::Clone() const {
    return static_cast<Controller*>(new ControllerFullState(*this));
}

void ControllerFullState::SetCoefficients(const double &coef_proportional, const double &coef_derivative, const double &coef_integral) {
    coef_proportional_ = coef_proportional;
    coef_derivative_ = coef_derivative;
    coef_integral_ = coef_integral;
}

Vector3D ControllerFullState::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D thrust;
    const double control_interval = (sensor_data[7] - latest_control_action_);
    for (unsigned int i = 0; i < 3; ++i) {
        const double error = target_position_[i] - sensor_data[i];
        const double derivative = (control_interval > 0.0 ? (error - previous_error_[i]) / control_interval : 0.0);
        integral_[i] += error * control_interval;
        double t = coef_proportional_ * error + coef_derivative_ * derivative + coef_integral_ * integral_[i];
        if (t > maximum_thrust_) {
            t = maximum_thrust_;
        } else if (t < -maximum_thrust_) {
            t = -maximum_thrust_;
        }
        thrust[i] = t;
        previous_error_[i] = error;
    }
    latest_control_action_ = sensor_data[7];
    return thrust;
}
