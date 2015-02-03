#ifndef RESULTWRITER_H
#define RESULTWRITER_H
#include "vector.h"
#include "asteroid.h"
#include "sensorsimulator.h"

#include <vector>
#include <string>
#include <fstream>

class FileWriter {
public:
    FileWriter();

    void CreateVisualizationFile(const std::string &path_to_file, const double &control_frequency,const Asteroid &asteroid, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &heights);

    void CreateSensorDataFile(const std::string &path_to_file, const unsigned int &random_seed, const double &interaction_interval, const double &simulation_time, const Asteroid &asteroid, const SystemState &system_state, const std::vector<std::vector<double> > &sensor_data);

private:
    std::ofstream file_;
};

#endif // RESULTWRITER_H
