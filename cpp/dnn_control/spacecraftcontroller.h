#ifndef SPACECRAFTCONTROLLER_H
#define SPACECRAFTCONTROLLER_H

#include "sensorsimulator.h"
#include "vector.h"

class SpacecraftController {
        /*
         * This abstract class generates the thrust for a given sensor_data input (which will hopefully result in hovering at some point).
        */
public:
    SpacecraftController(const int &dimensions);
    virtual ~SpacecraftController();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual void GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust) = 0;

    int Dimensions() const;

protected:
    // How large can the sensor space be
    int dimensions_;
};

#endif // SPACECRAFTCONTROLLER_H
