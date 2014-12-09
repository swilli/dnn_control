#include "sensorsimulator.h"
#include "utility.h"
#include <math.h>

SensorSimulator::SensorSimulator(const Asteroid &asteroid, const double &sensor_noise) : asteroid_(asteroid), normal_distribution_(boost::mt19937(time(0)),boost::normal_distribution<>(0.0, sensor_noise)) {
    dimensions_ = 0;
}

SensorSimulator::~SensorSimulator()
{

}

int SensorSimulator::Dimensions() const
{
    return dimensions_;
}
