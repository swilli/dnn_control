#ifndef TRAININGDATAGENERATOR_H
#define TRAININGDATAGENERATOR_H

#include "systemstate.h"
#include "samplefactory.h"
#include "asteroid.h"

#include <boost/tuple/tuple.hpp>

class TrainingDataGenerator {
    /*
    * This class generates the artificial sensor data required for a neural network controller which uses an Autoencoder to compress this generated sensor data.
    */
public:
    // The number of output dimensions the TrainingDataGenerator will generate
    static const unsigned int kDimensions;

    TrainingDataGenerator(SampleFactory &sample_factory, const Asteroid &asteroid);


    // Generates sensor data based on the current spacecraft state "state" and time "time", also returns the "labels", i.e., the ground truth used to generate the data
    boost::tuple<std::vector<double>, std::vector<double> > Generate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time);

    // As the TrainingDataGenerator can generate histories of sensor streams, resetting will reset the previous history
    void Reset();

private:
    // How large is the sensor data space
    unsigned int dimensions_;

    // The underlying random sample factory
    SampleFactory &sample_factory_;

    // The system's asteroid
    const Asteroid &asteroid_;

    // The history is implemented with a circular buffer. This index points to the first element in the history.
    unsigned int cache_index_;

    // The number of calls to Generate since the last call to Reset or the initial construction
    unsigned int num_generate_calls_;

    // The history of sensor data values
    std::vector<double> sensor_values_cache_;

    // The noise configuration for every sensor dimension
    std::vector<double> noise_configurations_;
};

#endif // TRAININGDATAGENERATOR_H
