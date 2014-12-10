#ifndef SENSORDATAGENERATOR_H
#define SENSORDATAGENERATOR_H

#include <string>

class SensorDataGenerator
{
public:
    SensorDataGenerator(const std::string &path_to_data_folder, const double &control_frequency, const double &data_set_time);

    void Generate(const int &num_datasets, const std::string &prefix="");

private:
    double data_set_time_;
    double control_frequency_;
    std::string path_to_data_folder_;
};

#endif // SENSORDATAGENERATOR_H
