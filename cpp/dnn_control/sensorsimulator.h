#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"
#include "systemstate.h"
#include "samplefactory.h"

#include <vector>

typedef std::vector<double> SensorData;
typedef std::vector<double> SensorNoiseConfiguration;

class SensorSimulator {
    /*
     * This abstract class generates the artificial the sensor data required for a controller.
     */
public:
    SensorSimulator(const unsigned int &dimensions, SampleFactory sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);

    virtual ~SensorSimulator();

    virtual SensorSimulator* Clone() const = 0;

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) = 0;

    unsigned int Dimensions() const;


    void SetSampleFactory(SampleFactory sample_factory);

protected:
    // How large is the sensor data space
    unsigned int dimensions_;

    SampleFactory sample_factory_;

    // The system's asteroid
    const Asteroid asteroid_;

    SensorNoiseConfiguration noise_configuration_;
};

#endif // SENSORSIMULATOR_H
