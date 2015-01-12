#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"
#include "odesystem.h"

#include <vector>

typedef std::vector<double> SensorData;

class SensorSimulator {
    /*
     * This abstract class generates the artificial the sensor data required for a controller.
     */
public:
    SensorSimulator(const unsigned int &dimensions, const Asteroid &asteroid);
    virtual ~SensorSimulator();


    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) = 0;

    unsigned int Dimensions() const;

protected:
    // The system's asteroid
    Asteroid asteroid_;

    // How large is the sensor data space
    unsigned int dimensions_;
};

#endif // SENSORSIMULATOR_H
