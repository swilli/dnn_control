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

    std::vector<double> sensor_seed(TDG_DATA_DIMENSIONS, 0.0);

    const Vector3D &position = {state[0], state[1], state[2]};
    const Vector3D &velocity = {state[3], state[4], state[5]};

    const double coef_norm_height = 1.0 / VectorNorm(height);

    const Vector3D &normalized_height = {height[0] * coef_norm_height, height[1] * coef_norm_height, height[2] * coef_norm_height};

    const double magn_velocity_parallel = VectorDotProduct(velocity, normalized_height);
    const Vector3D &velocity_parallel = {magn_velocity_parallel * normalized_height[0], magn_velocity_parallel * normalized_height[1], magn_velocity_parallel * normalized_height[2]};
    const Vector3D velocity_perpendicular = VectorSub(velocity, velocity_parallel);

    for (unsigned int i = 0; i < 3; ++i) {
        sensor_seed[i] = 100000.0 * velocity_parallel[i] * coef_norm_height;
        sensor_seed[3+i] = 100000.0 * velocity_perpendicular[i] * coef_norm_height;
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
        sensor_duplicates[i] = Normalize(sensor_duplicates[i], sensor_maximum_absolute_range);
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

    return boost::make_tuple(sensor_data, sensor_seed);
}

double TrainingDataGenerator::Normalize(const double &sensor_value, const double &max_abs_sensor_value) {
    double normalized_sensor_value = sensor_value;

#if TDG_NORMALIZE_SENSOR_VALUES
    if (normalized_sensor_value > max_abs_sensor_value) {
        normalized_sensor_value = max_abs_sensor_value;
    } else if (normalized_sensor_value < -max_abs_sensor_value) {
        normalized_sensor_value = -max_abs_sensor_value;
    }
    normalized_sensor_value = normalized_sensor_value * 0.5 / max_abs_sensor_value + 0.5;

#endif

    return normalized_sensor_value;
}
