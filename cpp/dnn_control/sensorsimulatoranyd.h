#ifndef SENSORSIMULATOR45D_H
#define SENSORSIMULATOR45D_H

#include "sensorsimulator.h"
#include "asteroid.h"

#include <vector>
#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/array.hpp>

#define SENSOR_SIMULATOR_DATA_DIMENSIONS  5
#define SENSOR_SIMULATOR_DATA_MULTIPLIER  3
#define SENSOR_SIMULATOR_DATA_HISTORY     3


class SensorSimulatorAnyD : public SensorSimulator
{
    /*
     * This class generates SENSOR_DATA_DIMENSIONS * SENSOR_DATA_MULTIPLIER sensor values out of SENSOR_DATA_DIMENSIONS initial sensor values.
     * In addition, it keeps the last SENSOR_DATA_HISTORY sensor readings. This gives a total dimension of SENSOR_DATA_DIMENSIONS * SENSOR_DATA_MULTIPLIER * SENSOR_DATA_HISTORY.
     */
public:
    typedef boost::array<double, SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER> SensorNoiseConfiguration;
    typedef boost::array<double, SENSOR_SIMULATOR_DATA_DIMENSIONS * SENSOR_SIMULATOR_DATA_MULTIPLIER * SENSOR_SIMULATOR_DATA_HISTORY> SensorDataCache;

    SensorSimulatorAnyD(const Asteroid &asteroid, const SensorNoiseConfiguration &configuration);
    virtual ~SensorSimulatorAnyD();

    virtual SensorData Simulate(const State &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

    // Resets first_simulation_ to false
    void ResetSimulator();

private:
    // Normal distributions for each sensor dimension
    std::vector<boost::variate_generator<boost::mt19937, boost::normal_distribution<> > > normal_distributions_;

    // Keep sensor data in a buffer
    SensorDataCache sensor_data_cache_;

    // True if nobody called Simulate on this simulator before
    bool first_simulation_;
};

#endif // SENSORSIMULATOR45D_H
