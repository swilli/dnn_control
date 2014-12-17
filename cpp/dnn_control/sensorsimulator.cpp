#include "sensorsimulator.h"
#include "utility.h"

SensorSimulator::SensorSimulator(const Asteroid &asteroid) : asteroid_(asteroid) {
    dimensions_ = 0;
}

SensorSimulator::~SensorSimulator()
{

}

int SensorSimulator::Dimensions() const
{
    return dimensions_;
}
