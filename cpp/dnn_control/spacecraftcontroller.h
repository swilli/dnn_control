#ifndef SPACECRAFTCONTROLLER_H
#define SPACECRAFTCONTROLLER_H

#include "sensorsimulator.h"
#include "vector.h"

class SpacecraftController
{
public:
    SpacecraftController();

    void GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust) const;
};

#endif // SPACECRAFTCONTROLLER_H
