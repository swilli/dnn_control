#ifndef SENSORSIMULATORFULLSTATE_H
#define SENSORSIMULATORFULLSTATE_H

#include "sensorsimulator.h"

class SensorSimulatorFullState : public SensorSimulator {
	/*
    * This class generates the artificial sensor data data which contains all information about the spacecraft state (except mass).
	* It generates relative position and velocity offsets.
    */
public:
	// The number of output dimensions the sensor simulator will generate
    static const unsigned int kDimensions;

    SensorSimulatorFullState(SampleFactory &sample_factory, const Asteroid &asteroid, const Vector3D &target_position);

    virtual ~SensorSimulatorFullState();

    // Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    virtual SensorData Simulate(const SystemState &state, const Vector3D  &, const Vector3D &, const double &time);

private:
	// The target position the spacecraft is supposed to hover over
    Vector3D target_position_;
};

#endif // SENSORSIMULATORFULLSTATE_H
