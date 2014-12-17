#include "sensordatagenerator.h"
#include "simulator.h"
#include "utility.h"
#include "sensorsimulatoranyd.h"
#include "fullstatecontroller.h"
#include "filewriter.h"

#include <ctime>
#include <sstream>

SensorDataGenerator::SensorDataGenerator(const std::string &path_to_data_folder, const double &control_frequency, const double &data_set_time) : data_set_time_(data_set_time), control_frequency_(control_frequency), path_to_data_folder_(path_to_data_folder) {
}

void SensorDataGenerator::Generate(const int &num_datasets, const std::string &prefix)
{
    for (int data_iter = 0; data_iter < num_datasets; ++data_iter) {
        const double time = data_set_time_;
        const Vector3D semi_axis = {SampleUniform(8000.0, 12000.0), SampleUniform(4000.0, 7500.0), SampleUniform(1000.0, 3500.0)};
        const double density = SampleUniform(1500.0, 3000.0);
        const Vector3D angular_velocity = {SampleSign() * SampleUniform(0.0002, 0.0008), 0.0, SampleSign() * SampleUniform(0.0002, 0.0008)};
        const double time_bias = 0.0;

        const Vector3D spacecraft_position = SamplePointOutSideEllipsoid(semi_axis, 4.0);

        Vector3D spacecraft_velocity = CrossProduct(angular_velocity, spacecraft_position);
        spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;

        const double spacecraft_specific_impulse = 200.0;
        const double spacecraft_mass = SampleUniform(500.0, 1500.0);

        const double control_frequency = control_frequency_;

        Vector3D target_position;
        for (int i = 0; i < 3; ++i) {
            target_position[i] = SampleUniform(spacecraft_position[i] - 500.0, spacecraft_position[i] + 500.0);
        }

        SensorNoiseConfiguration sensor_noise;
        for (unsigned int i = 0; i < sensor_noise.size(); ++i) {
            sensor_noise[i] = 0.05;
        }
        const double perturbation_noise = 1e-7;
        const double control_noise = 0.05;

        Asteroid asteroid(semi_axis, density, angular_velocity, time_bias);
        SensorSimulator *sensor_simulator = new SensorSimulatorAnyD(asteroid, sensor_noise);
        SpacecraftController *spacecraft_controller = new FullStateController(control_frequency, target_position);

        Simulator simulator(asteroid, sensor_simulator, spacecraft_controller, control_frequency, perturbation_noise, control_noise);
        simulator.InitSpacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse);

        const boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<SensorData> > result = simulator.Run(time, true);
        const std::vector<SensorData> sensor_data = boost::get<3>(result);

        std::string path_to_file(path_to_data_folder_);

        if (prefix == "") {
            time_t raw_time;
            struct tm *time_info;
            char buffer[80];

            std::time(&raw_time);
            time_info = localtime(&raw_time);
            strftime(buffer,80,"%d_%m_%H_%I_%M_%S_",time_info);

            std::string str(buffer);
            path_to_file += str;
        } else {
            path_to_file += prefix + "_";
        }
        path_to_file += "gen";
        std::stringstream ss;
        ss << data_iter + 1;
        path_to_file += ss.str();
        path_to_file += ".txt";

        std::cout << (data_iter + 1) << ": writing sensor data to file " << path_to_file << " ... ";
        FileWriter writer;
        writer.CreateSensorDataFile(path_to_file, control_frequency, time, asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse, target_position, sensor_data);
        std::cout << "done." << std::endl;
    }
}
