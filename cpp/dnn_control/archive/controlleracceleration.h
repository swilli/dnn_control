#ifndef CONTROLLERACCELEROMETER_H
#define CONTROLLERACCELEROMETER_H

#include "controller.h"

class ControllerAcceleration : public Controller
{
public:
    ControllerAcceleration(const double &control_frequency, const double &maximum_thrust, const Vector3D &target_position, const Vector3D &initial_position, const Vector3D &initial_velocity);
    ~ControllerAcceleration();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

private:
    SystemState state_;

    // PD members
    double control_interval_;
    double control_interval_pow2_;

    double constant_proportional_;

    double constant_derivative_;

    double constant_integral_;

    Vector3D target_position_;

    Vector3D previous_error_;

    Vector3D integral_;
};

#endif // CONTROLLERACCELEROMETER_H
