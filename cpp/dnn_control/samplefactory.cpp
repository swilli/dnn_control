#include "samplefactory.h"

boost::mt19937 SampleFactory::generator_;
boost::random::uniform_real_distribution<> SampleFactory::uniform_distribution_;

SampleFactory::SampleFactory() {

}

SampleFactory::~SampleFactory() {

}

void SampleFactory::Init(const unsigned int &random_seed) {
    generator_ = boost::mt19937(random_seed);
    uniform_distribution_ = boost::random::uniform_real_distribution<>(0.0, 1.0);
}

boost::random::mt19937& SampleFactory::RandomNumberGenerator() {
    return generator_;
}

double SampleFactory::SampleUniform(const double &minimum, const double &maximum) {
    return minimum + (maximum - minimum) * uniform_distribution_(generator_);
}

double SampleFactory::SampleSign() {
    const double value = SampleUniform(-1.0, 1.0);
    return value >= 0.0 ? 1.0 : -1.0;
}
