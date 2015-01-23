#include "sensorsimulator.h"

SensorSimulator::SensorSimulator(const unsigned int &dimensions, SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : dimensions_(dimensions), sample_factory_(sample_factory), asteroid_(asteroid), noise_configuration_(configuration) {

}

SensorSimulator::~SensorSimulator() {

}

unsigned int SensorSimulator::Dimensions() const {
    return dimensions_;
}
