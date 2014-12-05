#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"
#include "odesystem.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

typedef double SensorData[5];

class SensorSimulator
{
    /*
     * This class generates the artificial the sensor data required for a controller.
     */
public:
    SensorSimulator(const Asteroid &asteroid, const double &sensor_noise);

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data);

private:
    // The system's asteroid
    const Asteroid &asteroid_;

    // normal_distribution_ ~ N(0, sensor_noise), whereas sensor_noise is given in the constructor
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // SENSORSIMULATOR_H
