#include "controllerfullstate.h"

const unsigned int ControllerFullState::kDimensions = 8;

ControllerFullState::ControllerFullState(const double &maximum_thrust, const Vector3D &target_position)
    : Controller(kDimensions, 3, maximum_thrust) {
    for (unsigned int i = 0; i < 3; ++i) {
        target_position_[i] = target_position[i];
        previous_error_[i] = 0.0;
        integral_[i] = 0.0;
    }

    latest_control_action_ = 0.0;

    coef_integral_ = 0.0;
    coef_derivative_ = 0.0;
    coef_proportional_ = 0.0;
}

ControllerFullState::~ControllerFullState() {

}

void ControllerFullState::SetCoefficients(const std::vector<double> &pid_coefficients) {
    if (pid_coefficients.size() != number_of_parameters_) {
        throw SizeMismatchException();
    }
    coef_proportional_ = pid_coefficients[0];
    coef_derivative_ = pid_coefficients[1];
    coef_integral_ = pid_coefficients[2];
}

Vector3D ControllerFullState::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D thrust;
    const double control_interval = (sensor_data[7] - latest_control_action_);
    for (unsigned int i = 0; i < 3; ++i) {
        const double error = target_position_[i] - sensor_data[i];
        double derivative = 0.0;
        if (control_interval > 0.0) {
            derivative = (error - previous_error_[i]) / control_interval;
            integral_[i] += error * control_interval;
        }
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
