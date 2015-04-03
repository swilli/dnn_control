#ifndef FILEWRITER_H
#define FILEWRITER_H

#include "sensorsimulator.h"

#include <vector>
#include <string>
#include <fstream>
#include <eigen3/Eigen/Dense>

class FileWriter {
public:
    FileWriter(const std::string &path_to_file);
    ~FileWriter();

    // Create a file which can be visualized using vtrajectory.py
    void CreateTrajectoryFile(const double &control_frequency, const Asteroid &asteroid, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &heights);

    // Create a file which can be used to train e.g., an autoencoder: The file will contain the stream of sensor data readings produced in a simulation
    void CreateSensorDataFile(const unsigned int &random_seed, const double &control_frequency, const double &simulation_time, const Asteroid &asteroid, const SystemState &system_state, const std::vector<Vector3D> &thrusts, const std::vector<std::vector<double> > &sensor_data);

    // Create a file which can be visualized using vevaluation.py
    void CreateEvaluationFile(const unsigned int &random_seed, const Vector3D &target_position, const Asteroid &asteroid, const std::vector<double> &times, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &velocities, const std::vector<Vector3D> &thrusts);

    // Create a file which can be visualized using vpostevaluation.py
    void CreatePostEvaluationFile(const std::vector<unsigned int> &random_seeds, const std::vector<double> &mean_errors, const std::vector<std::pair<double, double> > &min_max_errors);

    // Create a file which can be visualized using vconvexity.py
    void CreateConvexityFile(const unsigned int &random_seed, const unsigned int &dimension, const std::vector<std::pair<double, double> > &fitness);

    // Create a file which can be visualized using vactionset.py
    void CreateActionSetFile(const std::vector<Vector3D> actions);

    // Create a file which can be loaded as an LSPI Policy
    void CreateLSPIWeightsFile(const Eigen::VectorXd &weights);

private:
    // The file to write into
    std::ofstream file_;
};

#endif // FILEWRITER_H
