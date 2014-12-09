#ifndef SENSORSIMULATOR_H
#define SENSORSIMULATOR_H

#include "asteroid.h"
#include "odesystem.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <vector>

typedef std::vector<double> SensorData;

class SensorSimulator {
    /*
     * This abstract class generates the artificial the sensor data required for a controller.
     */
public:
    SensorSimulator(const Asteroid &asteroid, const double &sensor_noise);
    virtual ~SensorSimulator();


    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual void Simulate(const State &state, const Vector3D &perturbations_acceleration, const double &time, SensorData &sensor_data) = 0;

    int Dimensions() const;

protected:
    // The system's asteroid
    const Asteroid &asteroid_;

    // How large is the sensor data space
    int dimensions_;

    // normal_distribution_ ~ N(0, sensor_noise), whereas sensor_noise is given in the constructor
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normal_distribution_;
};

#endif // SENSORSIMULATOR_H
