#ifndef CONTROLLERANYD_H
#define CONTROLLERANYD_H

#include "controller.h"

#define CONTROLLER_SENSOR_DATA_DIMENSIONS  45

class ControllerAnyD : public Controller
{
public:
    ControllerAnyD(const double &control_frequency, const double &maximum_thrust);
    ~ControllerAnyD();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);
};

#endif // CONTROLLERANYD_H
