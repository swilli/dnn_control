#ifndef CONTROLLER5D_H
#define CONTROLLER5D_H

#include "spacecraftcontroller.h"

class Controller5D : public SpacecraftController
{
public:
    Controller5D();
    virtual ~Controller5D();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual void GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust);
};

#endif // CONTROLLER5D_H
