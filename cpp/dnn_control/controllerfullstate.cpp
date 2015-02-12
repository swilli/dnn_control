#include "controllerfullstate.h"

#if CFS_WITH_VELOCITY
const unsigned int ControllerFullState::kDimensions = 7;
#else
const unsigned int ControllerFullState::kDimensions = 4;
#endif

ControllerFullState::ControllerFullState(const double &maximum_thrust, const Vector3D &target_position)
    : Controller(kDimensions, kDimensions - 1, maximum_thrust) {
    for (unsigned int i = 0; i < 3; ++i) {
        target_state_[i] = target_position[i];
        previous_error_[i] = 0.0;
        integral_[i] = 0.0;
#if CFS_WITH_VELOCITY
        target_state_[3+i] = 0.0;
        previous_error_[3+i] = 0.0;
        integral_[3+i] = 0.0;
#endif
    }

    latest_control_action_ = 0.0;
}

ControllerFullState::~ControllerFullState() {

}

void ControllerFullState::SetCoefficients(const std::vector<double> &pid_coefficients) {
    if (pid_coefficients.size() != number_of_parameters_) {
        throw SizeMismatchException();
    }
    for (unsigned int i = 0; i < pid_coefficients_.size(); ++i) {
        pid_coefficients_.at(i) = pid_coefficients.at(i);
    }
}

Vector3D ControllerFullState::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D thrust;
    const double control_interval = (sensor_data[dimensions_ - 1] - latest_control_action_);

    for (unsigned int i = 0; i < 3; ++i) {
        const double error_position = target_state_[i] - sensor_data[i];
        double derivative_position = 0.0;
        if (control_interval > 0.0) {
            derivative_position = (error_position - previous_error_[i]) / control_interval;
            integral_[i] += error_position * control_interval;
        }
#if CFS_WITH_VELOCITY
        const double error_velocity = target_state_[3+i] - sensor_data[3+i];
        double derivative_velocity = 0.0;
        if (control_interval > 0.0) {
            derivative_velocity = (error_velocity - previous_error_[3+i]) / control_interval;
            integral_[3+i] += error_velocity * control_interval;
        }
        double t = pid_coefficients_.at(0) * error_position + pid_coefficients_.at(1) * derivative_position + pid_coefficients_.at(2) * integral_[i]
                + pid_coefficients_.at(3) * error_velocity + pid_coefficients_.at(4) * derivative_velocity + pid_coefficients_.at(5) * integral_[3+i];
#else
        double t = pid_coefficients_.at(0) * error_position + pid_coefficients_.at(1) * derivative_position + pid_coefficients_.at(2) * integral_[i];
#endif
        if (t > maximum_thrust_) {
            t = maximum_thrust_;
        } else if (t < -maximum_thrust_) {
            t = -maximum_thrust_;
        }
        thrust[i] = t;
        previous_error_[i] = error_position;
#if CFS_WITH_VELOCITY
        previous_error_[3+i] = error_velocity;
#endif
    }
    latest_control_action_ = sensor_data[dimensions_ - 1];

    return thrust;
}
