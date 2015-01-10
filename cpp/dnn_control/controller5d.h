#ifndef CONTROLLER5D_H
#define CONTROLLER5D_H

#include "controller.h"

class Controller5D : public Controller
{
public:
    Controller5D(const double &maximum_thrust);
    virtual ~Controller5D();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);
};

#endif // CONTROLLER5D_H
