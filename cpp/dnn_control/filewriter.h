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

    // Create a file which can be visualized using vtrajectory.py
    void CreateVisualizationFile(const double &control_frequency, const Asteroid &asteroid, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &heights);

    // Create a file which can be used to train e.g., an autoencoder: The file will contain the stream of sensor data readings produced in a simulation
    void CreateSensorDataFile(const unsigned int &random_seed, const double &control_frequency, const double &simulation_time, const Asteroid &asteroid, const SystemState &system_state, const std::vector<SensorData> &sensor_data);

    // Create a file which can be visualized using vevaluation.py
    void CreateEvaluationFile(const unsigned int &random_seed, const Vector3D &target_position, const std::vector<double> &times, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &velocities, const std::vector<Vector3D> &thrusts);

    // Create a file which can be visualized using vpostevaluation.py
    void CreatePostEvaluationFile(const std::vector<unsigned int> &random_seeds, const std::vector<std::vector<double> > &mean_errors, const std::vector<std::vector<std::pair<double, double> > > &min_max_errors);

    // Create a file which can be visualized using vconvexity.py
    void CreateConvexityFile(const unsigned int &random_seed, const unsigned int &dimension, const std::vector<std::pair<double, double> > &fitness);

    // Create a file which can be visualized using vactionset.py
    void CreateActionSetFile(const std::vector<Vector3D> actions);

private:
	// The file to write into
    std::ofstream file_;
};

#endif // FILEWRITER_H
