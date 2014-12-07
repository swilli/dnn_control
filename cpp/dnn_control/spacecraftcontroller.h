#ifndef SPACECRAFTCONTROLLER_H
#define SPACECRAFTCONTROLLER_H

#include "sensorsimulator.h"
#include "vector.h"

class SpacecraftController {
        /*
         * This class generates the thrust for a given sensor_data input (which will hopefully result in hovering at some point).
        */
public:
    SpacecraftController();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    void GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust) const;
};

#endif // SPACECRAFTCONTROLLER_H
