#include "sensordatagenerator.h"
#include "simulator.h"
#include "utility.h"
#include "sensorsimulatoranyd.h"
#include "controllerfullstate.h"
#include "filewriter.h"

#include <ctime>
#include <sstream>

SensorDataGenerator::SensorDataGenerator(const std::string &path_to_output_folder, const double &control_frequency, const double &data_set_time) : data_set_time_(data_set_time), control_frequency_(control_frequency), path_to_output_folder_(path_to_output_folder) {

}

void SensorDataGenerator::Generate(const unsigned int &num_datasets, const bool &controlled, const std::string &prefix) {
    for (unsigned int data_iter = 0; data_iter < num_datasets; ++data_iter) {
        const double time = data_set_time_;

        const Vector3D semi_axis = {SampleUniform(8000.0, 12000.0), SampleUniform(4000.0, 7500.0), SampleUniform(1000.0, 3500.0)};
        const double density = SampleUniform(1500.0, 3000.0);
        Vector2D angular_velocity_xz = {SampleSign() * SampleUniform(0.0002, 0.0008), SampleSign() * SampleUniform(0.0002, 0.0008)};
        const double time_bias = SampleUniform(0.0, 24.0 * 60 * 60);

        Asteroid asteroid(semi_axis, density, angular_velocity_xz, time_bias);

        const Vector3D spacecraft_position = SamplePointOutSideEllipsoid(semi_axis, 4.0);

        Vector3D spacecraft_velocity;
        if (controlled) {
            for (unsigned int i = 0; i < 3; ++i) {
                spacecraft_velocity[i] = SampleUniform(-0.3,0.3);
            }
        } else {
            const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(0.0));
            spacecraft_velocity = CrossProduct(angular_velocity, spacecraft_position);
            spacecraft_velocity[0] *= -1; spacecraft_velocity[1] *= -1; spacecraft_velocity[2] *= -1;
        }


        const double spacecraft_specific_impulse = 200.0;
        const double spacecraft_mass = SampleUniform(450.0, 550.0);
        const double spacecraft_maximum_thrust = 21.0;

        const double control_frequency = control_frequency_;

        Vector3D target_position;
        for (unsigned int i = 0; i < 3; ++i) {
            target_position[i] = SampleUniform(spacecraft_position[i] - 3.0, spacecraft_position[i] + 3.0);
        }

        SensorSimulatorAnyD::SensorNoiseConfiguration sensor_noise;
        for (unsigned int i = 0; i < sensor_noise.size(); ++i) {
            sensor_noise[i] = 0.05;
        }
        const double perturbation_noise = 1e-7;
        const double control_noise = 0.05;

        SensorSimulator *sensor_simulator = new SensorSimulatorAnyD(asteroid, sensor_noise);
        ControllerFullState *default_controller = NULL;
        if (controlled) {
            default_controller = new ControllerFullState(control_frequency, spacecraft_maximum_thrust, target_position);
        }
        Simulator simulator(control_frequency, perturbation_noise, control_noise, asteroid, sensor_simulator, NULL, default_controller);
        simulator.InitSpacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse);

        std::cout << "generation " << (data_iter + 1) << " : " << std::endl;
        std::cout << "   running simulation ... ";
        const boost::tuple<double, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<SensorData> > result = simulator.Run(time, true);
        std::cout << "done." << std::endl;
        const std::vector<Vector3D> positions = boost::get<1>(result);
        const std::vector<Vector3D> heights = boost::get<2>(result);
        const std::vector<SensorData> sensor_data = boost::get<3>(result);

        std::string path_to_sensor_data_file(path_to_output_folder_);

        if (prefix == "") {
            time_t raw_time;
            struct tm *time_info;
            char buffer[80];

            std::time(&raw_time);
            time_info = localtime(&raw_time);
            strftime(buffer,80,"%d_%m_%H_%I_%M_%S_",time_info);

            std::string str(buffer);
            path_to_sensor_data_file += str;
        } else {
            path_to_sensor_data_file += prefix + "_";
        }
        if (controlled) {
            path_to_sensor_data_file += "controlled_";
        } else {
            path_to_sensor_data_file += "uncontrolled_";
        }
        path_to_sensor_data_file += "gen";
        std::stringstream ss;
        ss << data_iter + 1;
        path_to_sensor_data_file += ss.str();

        std::string path_to_trajectory_file(path_to_sensor_data_file);
        path_to_trajectory_file += "_trajectory.txt";

        path_to_sensor_data_file += ".txt";

        std::cout << "   writing sensor data to file " << path_to_sensor_data_file << " ... ";
        FileWriter writer;
        writer.CreateSensorDataFile(path_to_sensor_data_file, control_frequency, time, asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse, target_position, sensor_data);
        std::cout << "done." << std::endl;

        std::cout << "   writing stats to file " << path_to_trajectory_file << " ... ";
        writer.CreateVisualizationFile(path_to_trajectory_file, control_frequency, asteroid, positions, heights);
        std::cout << "done." << std::endl;
    }
}
