#ifndef FULLSTATECONTROLLER_H
#define FULLSTATECONTROLLER_H

#include "controller.h"
#include "sensorsimulatorfullstate.h"

class ControllerFullState : public Controller
{
public:
    ControllerFullState(const double &control_frequency, const double &maximum_thrust, const Vector3D &target_position);
    virtual ~ControllerFullState();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

private:
    double control_interval_;

    double constant_proportional_;

    double constant_derivative_;

    double constant_integral_;

    Vector3D target_position_;

    Vector3D previous_error_;

    Vector3D integral_;
};

#endif // FULLSTATECONTROLLER_H
