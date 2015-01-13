#include "controlleracceleration.h"

ControllerAcceleration::ControllerAcceleration(const double &control_frequency, const double &maximum_thrust, const Vector3D &target_position, const Vector3D &initial_position, const Vector3D &initial_velocity) : Controller(4, maximum_thrust) {
    for (unsigned int i = 0; i < 3; ++i) {
        target_position_[i] = target_position[i];
        previous_error_[i] = 0.0;
        integral_[i] = 0.0;
        state_[i] = initial_position[i];
        state_[3+i] = initial_velocity[i];
    }

    control_interval_ = 1.0 / control_frequency;
    control_interval_pow2_ = control_interval_ * control_interval_;

    constant_integral_ = 0.0;
    constant_derivative_ = 5;
    constant_proportional_ = 4;
}

ControllerAcceleration::~ControllerAcceleration() {

}

Vector3D ControllerAcceleration::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D estimated_position;
    estimated_position[0] = state_[0];
    estimated_position[1] = state_[1];
    estimated_position[2] = state_[2];

    const double &mass = sensor_data[3];


    Vector3D thrust_acceleration;
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        const double error = target_position_[i] - estimated_position[i];
        const double derivative = (error - previous_error_[i]) / control_interval_;
        integral_[i] += error * control_interval_;
        double t = constant_proportional_ * error + constant_derivative_ * derivative + constant_integral_ * integral_[i];
        if (t > maximum_thrust_) {
            t = maximum_thrust_;
        } else if (t < -maximum_thrust_) {
            t = -maximum_thrust_;
        }
        thrust[i] = t;
        thrust_acceleration[i] = t / mass;
        previous_error_[i] = error;
    }

    const Vector3D accelerations = {sensor_data[0] + thrust_acceleration[0],
                                    sensor_data[1] + thrust_acceleration[1],
                                    sensor_data[2] + thrust_acceleration[2]};

    // Propagate (integrate) state
    for (unsigned int i = 0; i < 3; ++i) {
        state_[i] += control_interval_ * state_[3+i] + 0.5 * control_interval_pow2_ * accelerations[i];
        state_[3+i] += control_interval_ * accelerations[i];
    }
    state_[6] = mass;

    return thrust;
}

