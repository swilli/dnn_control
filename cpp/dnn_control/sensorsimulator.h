#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"
#include "systemstate.h"
#include "samplefactory.h"

#include <vector>

typedef std::vector<double> SensorData;

class SensorSimulator {
    /*
     * This abstract class generates the artificial the sensor data required for a controller.
     */
public:
    SensorSimulator(const unsigned int &dimensions, SampleFactory &sample_factory, const Asteroid &asteroid);

    virtual ~SensorSimulator();

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) = 0;

    unsigned int Dimensions() const;

    class Exception {};

protected:
    // How large is the sensor data space
    unsigned int dimensions_;

    // The underlying random sample factory
    SampleFactory &sample_factory_;

    // The system's asteroid
    const Asteroid &asteroid_;

    // The noise configuration for every sensor dimension
    std::vector<double> noise_configurations_;
};

#endif // SENSORSIMULATOR_H
