#include "sensorsimulator.h"
#include "utility.h"

SensorSimulator::SensorSimulator(const unsigned int &dimensions, const Asteroid &asteroid) : asteroid_(asteroid), dimensions_(dimensions) {

}

SensorSimulator::~SensorSimulator() {

}

unsigned int SensorSimulator::Dimensions() const {
    return dimensions_;
}
