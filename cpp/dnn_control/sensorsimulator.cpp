#include "sensorsimulator.h"

SensorSimulator::SensorSimulator(const unsigned int &dimensions, SampleFactory &sample_factory, const Asteroid &asteroid, const SensorNoiseConfiguration &configuration) : asteroid_(asteroid), dimensions_(dimensions), sample_factory_(sample_factory), noise_configuration_(configuration) {

}

SensorSimulator::~SensorSimulator() {

}

unsigned int SensorSimulator::Dimensions() const {
    return dimensions_;
}
