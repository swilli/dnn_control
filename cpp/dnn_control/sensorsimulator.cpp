#include "sensorsimulator.h"

SensorSimulator::SensorSimulator(const unsigned int &dimensions, SampleFactory &sample_factory, const Asteroid &asteroid) : asteroid_(asteroid), dimensions_(dimensions), sample_factory_(sample_factory) {

}

SensorSimulator::~SensorSimulator() {

}

SensorNoiseConfiguration SensorSimulator::NoiseConfiguration() const {
    return noise_configuration_;
}

unsigned int SensorSimulator::Dimensions() const {
    return dimensions_;
}
