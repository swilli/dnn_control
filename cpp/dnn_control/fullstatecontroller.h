#ifndef FULLSTATECONTROLLER_H
#define FULLSTATECONTROLLER_H

#include "spacecraftcontroller.h"
#include "fullstatesensorsimulator.h"

class FullStateController : public SpacecraftController
{
public:
    FullStateController(const double &control_frequency, const Vector3D &target_position);
    virtual ~FullStateController();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual void GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust);

protected:
    double control_interval_;

    double constant_proportional_;

    double constant_derivative_;

    double constant_integral_;

    Vector3D target_position_;

    Vector3D previous_error_;

    Vector3D integral_;
};

#endif // FULLSTATECONTROLLER_H
