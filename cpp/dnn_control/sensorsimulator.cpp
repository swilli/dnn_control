#include "sensorsimulator.h"

SensorSimulator::SensorSimulator(const unsigned int &dimensions, SampleFactory &sample_factory, const Asteroid &asteroid)
    : dimensions_(dimensions), sample_factory_(sample_factory), asteroid_(asteroid) {

}

SensorSimulator::~SensorSimulator() {

}

unsigned int SensorSimulator::Dimensions() const {
    return dimensions_;
}
