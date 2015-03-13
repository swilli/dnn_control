#include "sensorsimulator.h"

#include <sstream>

SensorSimulator::SensorSimulator(const unsigned int &dimensions, SampleFactory &sample_factory, const Asteroid &asteroid)
    : dimensions_(dimensions), sample_factory_(sample_factory), asteroid_(asteroid) {

}

SensorSimulator::~SensorSimulator() {

}

unsigned int SensorSimulator::Dimensions() const {
    return dimensions_;
}

std::string SensorSimulator::SensorDataToString(const std::vector<double> &data) {
    std::stringstream result;
    if (data.size()) {
        result << data[0];
    }
    for (unsigned int i = 1; i < data.size(); ++i) {
        result << ",\t" << data[i];
    }
    return result.str();
}
