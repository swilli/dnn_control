#include "filewriter.h"
#include <iomanip>

FileWriter::FileWriter() {
    file_ << std::setprecision(10);
}

void FileWriter::CreateVisualizationFile(const std::string &path_to_file, const double &control_frequency, const Asteroid &asteroid, const std::vector<Vector3D> &positions, const std::vector<Vector3D> &heights) {
    file_.open(path_to_file.c_str());
    const Vector3D semi_axis = asteroid.SemiAxis();
    const Vector2D angular_velocity_xz = asteroid.ConstructorAngularVelocitiesXZ();
    file_ << semi_axis[0] << ",\t" << semi_axis[1] << ",\t" << semi_axis[2] << ",\t" << asteroid.Density() << ",\t" << angular_velocity_xz[0] << ",\t" << angular_velocity_xz[1] << ",\t" << asteroid.TimeBias() << ",\t" << control_frequency << std::endl;
    for (unsigned int i = 0; i < positions.size(); ++i) {
        const Vector3D &pos = positions.at(i);
        const Vector3D &height = heights.at(i);
        file_ << pos[0] << ",\t" << pos[1] << ",\t" << pos[2] << ",\t" << height[0] << ",\t" << height[1] << ",\t" << height[2] << " \n";
    }
    file_.close();
}

void FileWriter::CreateSensorDataFile(const std::string &path_to_file, const unsigned int &random_seed, const double &interaction_interval, const double &simulation_time, const Asteroid &asteroid,
                                      const SystemState &system_state, const std::vector<std::vector<double> > &sensor_data)
{
    const Vector2D angular_velocity = asteroid.ConstructorAngularVelocitiesXZ();
    const Vector3D semi_axis = asteroid.SemiAxis();
    file_.open(path_to_file.c_str());
    file_ << "# interaction interval: " << interaction_interval << " Hz" << std::endl;
    file_ << "# simulation time: " << simulation_time << " s" << std::endl;
    file_ << "#" << std::endl;
    file_ << "# asteroid:" << std::endl;
    file_ << "#  density: " << asteroid.Density() << " kg/m^3" << std::endl;
    file_ << "#  time bias: " << asteroid.TimeBias() << " s" << std::endl;
    file_ << "#  semi axis: " << semi_axis[0] << ",\t" << semi_axis[1] << ",\t" << semi_axis[2] <<" m" << std::endl;
    file_ << "#  angular velocity x,z: " << angular_velocity[0] << ", " << angular_velocity[1] << " 1/s" << std::endl;
    file_ << "#" << std::endl;
    file_ << "# spacecraft:" << std::endl;
    file_ << "#  mass: " << system_state[6] << " kg" << std::endl;
    file_ << "#  position: " << system_state[0] << ", " << system_state[1] << ", " << system_state[2] << " m" << std::endl;
    file_ << "#  velocity: " << system_state[3] << ", " << system_state[4] << ", " << system_state[5] << " m/s" << std::endl;
    file_ << "#" << std::endl;

    for (unsigned int i = 0; i < sensor_data.size(); ++i) {
        const SensorData &data = sensor_data.at(i);
        file_ << data[0];
        for (unsigned int j = 1; j < data.size(); ++j) {
            file_ << ", " << data[j];
        }
        file_ << std::endl;
    }

    file_.close();
}


