#ifndef SENSORSIMULATORPARTIALSTATE_H
#define SENSORSIMULATORPARTIALSTATE_H

#include "sensorsimulator.h"
#include "configuration.h"

class SensorSimulatorPartialState : public SensorSimulator {
    /*
    * This class generates the artificial sensor data which contains not all information about the spacecraft state.
    * The data produced by this sensor simulator contains optical flow and accelerometer data.
    */
public:
    // The number of output dimensions the sensor simulator will generate
    static const unsigned int kDimensions;

    SensorSimulatorPartialState(SampleFactory &sample_factory, const Asteroid &asteroid);


    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual std::vector<double> Simulate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

private:

    static double Standardize(const double &sensor_value, const std::pair<double, double> &mean_std_values);

#if PGMOS_ENABLE_DIRECTION_SENSOR
    Vector3D surface_point_;
#endif
};

#endif // SENSORSIMULATORPARTIALSTATE_H
