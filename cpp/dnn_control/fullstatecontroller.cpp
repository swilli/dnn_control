#include "fullstatecontroller.h"

FullStateController::FullStateController(const double &control_frequency, const Vector3D &target_position) : SpacecraftController(7),
    control_interval_(1.0 / control_frequency) {
    for (int i = 0; i < 3; ++i) {
        target_position_[i] = target_position[i];
        previous_error_[i] = 0.0;
        integral_[i] = 0.0;
    }

    constant_integral_ = 0.0;
    constant_derivative_ = 5;
    constant_proportional_ = 4;
}

FullStateController::~FullStateController() {

}

void FullStateController::GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust) {
    for (int i = 0; i < 3; ++i) {
        const double error = target_position_[i] - sensor_data[i];
        const double derivative = (error - previous_error_[i]) / control_interval_;
        integral_[i] += error * control_interval_;
        thrust[i] = 0.0; //constant_proportional_ * error + constant_derivative_ * derivative + constant_integral_ * integral_[i];
        previous_error_[i] = error;
    }
}
