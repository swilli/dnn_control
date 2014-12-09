#ifndef FULLSTATECONTROLLER_H
#define FULLSTATECONTROLLER_H

#include "spacecraftcontroller.h"
#include "fullstatesensorsimulator.h"

class FullStateController : public SpacecraftController
{
public:
    FullStateController(const Vector3D &target_position);
    virtual ~FullStateController();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual void GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust);

protected:
    Vector3D target_position_;
};

#endif // FULLSTATECONTROLLER_H
