#include "trainingdatagenerator.h"
#include "configuration.h"

const unsigned int TrainingDataGenerator::kDimensions = TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER * (TDG_DATA_HISTORY + 1);

TrainingDataGenerator::TrainingDataGenerator(SampleFactory &sample_factory, const Asteroid &asteroid)
    : dimensions_(kDimensions), sample_factory_(sample_factory), asteroid_(asteroid) {

    noise_configurations_ = std::vector<double>(TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER, 0.05);

    sensor_values_cache_ = std::vector<double>(TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER * (TDG_DATA_HISTORY + 1), 0.0);
    cache_index_ = 0;
    num_generate_calls_ = 0;
}

void TrainingDataGenerator::Reset() {
    cache_index_ = 0;
    num_generate_calls_ = 0;
}


boost::tuple<std::vector<double>, std::vector<double> > TrainingDataGenerator::Generate(const SystemState &state, const Vector3D &height, const Vector3D &perturbations_acceleration, const double &time) {
    std::vector<double> sensor_data(dimensions_, 0.0);
    std::vector<double> labels(TDG_DATA_DIMENSIONS, 0.0);

    std::vector<double> sensor_seed(TDG_DATA_DIMENSIONS, 0.0);

    const Vector3D &velocity = {state[3], state[4], state[5]};

    const double norm_height_pow2 = VectorDotProduct(height, height);
    const double norm_height = sqrt(norm_height_pow2);
    const double coef_norm_height = sqrt(3) / norm_height;

    const double velocity_dot_height = VectorDotProduct(velocity, height);
    const double scaling = velocity_dot_height / norm_height_pow2;

    const Vector3D &velocity_vertical = {scaling * height[0], scaling * height[1], scaling * height[2]};
    const Vector3D velocity_horizontal = VectorSub(velocity, velocity_vertical);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_seed[i] = velocity_vertical[i] * coef_norm_height;
        sensor_seed[3+i] = velocity_horizontal[i] * coef_norm_height;
    }

    for (unsigned int i = 0; i < TDG_DATA_DIMENSIONS; ++i) {
        labels[i] = sensor_seed[i];
    }

    // Every initial sensor value gets tripled and added with normal distributed noise
    std::vector<double> sensor_duplicates(TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER, 0.0);
    for (unsigned int i = 0; i < TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER; ++i) {
        const unsigned int sensor_data_index = i % TDG_DATA_DIMENSIONS;
        sensor_duplicates[i] = sensor_seed[sensor_data_index];

#if TDG_DATA_WITH_NOISE
        sensor_duplicates[i] += sensor_seed[sensor_data_index] * sample_factory_.SampleNormal(0.0, noise_configurations_.at(i));
#endif

        // Normalize between [0,1]
        const double sensor_maximum_absolute_range = 1e-3;
        if (sensor_duplicates[i] > sensor_maximum_absolute_range) {
            sensor_duplicates[i] = sensor_maximum_absolute_range;
        } else if (sensor_duplicates[i] < -sensor_maximum_absolute_range) {
            sensor_duplicates[i] = -sensor_maximum_absolute_range;
        }
        sensor_duplicates[i] = 0.5 * sensor_duplicates[i] / sensor_maximum_absolute_range + 0.5;
    }

       // write new sensor values at correct place
        const int cache_index = cache_index_;
        int start_index = cache_index - (TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER);
        if (start_index < 0) {
            start_index = dimensions_ - (TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER);
        }
        for (unsigned int i = 0; i < TDG_DATA_DIMENSIONS * TDG_DATA_MULTIPLIER; ++i) {
            sensor_values_cache_[start_index +i] = sensor_duplicates[i];
        }

    for (unsigned int i = 0; i < dimensions_; ++i) {
        sensor_data[(dimensions_ - 1) - i] = sensor_values_cache_[(cache_index_ + i) % dimensions_];
    }

    cache_index_ += TDG_DATA_DIMENSIONS*TDG_DATA_MULTIPLIER;
    if (cache_index_ == dimensions_) {
        cache_index_ = 0;
    }

    if (num_generate_calls_ < TDG_DATA_HISTORY) {
        num_generate_calls_++;
        return std::vector<double>(dimensions_, 0.0);
    }

    return boost::make_tuple(sensor_data, labels);
}
