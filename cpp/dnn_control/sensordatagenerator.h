#ifndef SENSORDATAGENERATOR_H
#define SENSORDATAGENERATOR_H

#include <string>

class SensorDataGenerator
{
public:
    SensorDataGenerator(const std::string &path_to_output_folder, const double &data_set_time);

    void Generate(const unsigned int &num_datasets, const unsigned int &random_seed=0, const std::string &prefix="sensor_stream");

private:
    double data_set_time_;
    std::string path_to_output_folder_;
};

#endif // SENSORDATAGENERATOR_H
