#ifndef SAMPLEFACTORY_H
#define SAMPLEFACTORY_H

#include <boost/random.hpp>

class SampleFactory
{
public:
    SampleFactory();
    ~SampleFactory();

    // Call before using the SampleFactory
    static void Init(const unsigned int &random_seed);

    // The underlying RNG
    static boost::mt19937 &RandomNumberGenerator();

    // X ~ U(minimum, maximum)
    static double SampleUniform(const double &minimum, const double &maximum);

    // X ~ U({-1,1})
    static double SampleSign();

private:
    static boost::mt19937 generator_;
    static boost::random::uniform_real_distribution<> uniform_distribution_;


};

#endif // SAMPLEFACTORY_H
