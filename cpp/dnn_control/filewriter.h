#ifndef FILEWRITER_H
#define FILEWRITER_H

#include "pagmosimulation.h"

#include <vector>
#include <string>
#include <fstream>

class FileWriter {
public:
    FileWriter(const std::string &path_to_file);
    ~FileWriter();

    void CreateVisualizationFile(const double &control_frequency, const Asteroid &asteroid, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &heights);

    void CreateSensorDataFile(const unsigned int &random_seed, const double &control_frequency, const double &simulation_time, const Asteroid &asteroid, const SystemState &system_state, const std::vector<SensorData> &sensor_data);

    void CreateEvaluationFile(const unsigned int &random_seed, const Vector3D &target_position, const std::vector<double> &times, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &velocities, const std::vector<Vector3D> &thrusts);

    void CreatePostEvaluationFile(const std::vector<unsigned int> &random_seeds, const std::vector<std::vector<double> > &fitness);

private:
    std::ofstream file_;
};

#endif // FILEWRITER_H
