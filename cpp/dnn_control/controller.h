#ifndef SPACECRAFTCONTROLLER_H
#define SPACECRAFTCONTROLLER_H

#include "sensorsimulator.h"
#include "vector.h"

class Controller {
        /*
         * This abstract class generates the thrust for a given sensor_data input (which will hopefully result in hovering at some point).
        */
public:
    Controller(const unsigned int &dimensions, const double &maximum_thrust);
    virtual ~Controller();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data) = 0;

    unsigned int Dimensions() const;

protected:
    // How large can the sensor space be
    unsigned int dimensions_;

    // What is the maximum absolute thrust that the spacecraft can generate
    double maximum_thrust_;
};

#endif // SPACECRAFTCONTROLLER_H
