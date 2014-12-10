#ifndef RESULTWRITER_H
#define RESULTWRITER_H
#include "vector.h"
#include "asteroid.h"
#include "sensorsimulator.h"

#include <vector>
#include <string>
#include <fstream>

class FileWriter
{
public:
    FileWriter();

    void CreateVisualizationFile(const std::string &path_to_file, const double &control_frequency,const Asteroid &asteroid, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &heights);

    void CreateSensorDataFile(const std::string &path_to_file, const double &control_frequency, const double &time, const Asteroid &asteroid, const Vector3D &spacecraft_position,
                              const Vector3D spacecraft_velocity, const double &spacecraft_mass, const double &spacecraft_specific_impulse, const Vector3D &target_position, const std::vector<SensorData> &sensor_data);

private:
    std::ofstream file_;
};

#endif // RESULTWRITER_H
