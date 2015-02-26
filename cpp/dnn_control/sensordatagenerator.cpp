#include "sensordatagenerator.h"
#include "filewriter.h"
#include "pagmosimulationneuralnetwork.h"
#include "samplefactory.h"

#include <ctime>
#include <sstream>

SensorDataGenerator::SensorDataGenerator(const std::string &path_to_output_folder, const double &data_set_time)
    : data_set_time_(data_set_time), path_to_output_folder_(path_to_output_folder) {

}

void SensorDataGenerator::Generate(const unsigned int &num_datasets, const unsigned int &random_seed) {
    SampleFactory sample_factory(random_seed);

    for (unsigned int data_iter = 0; data_iter < num_datasets; ++data_iter) {
        std::cout << "generation " << (data_iter + 1) << " : " << std::endl;
        PaGMOSimulationNeuralNetwork simulation(sample_factory.SampleRandomInteger(), data_set_time_);

        std::cout << "   running simulation ... ";
        const boost::tuple<std::vector<std::vector<double> >, std::vector<std::vector<double> >, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.GenerateSensorDataSet();
        const std::vector<std::vector<double> > &data_set = boost::get<0>(result);
        const std::vector<std::vector<double> > &labels = boost::get<1>(result);
        const std::vector<Vector3D> &positions = boost::get<2>(result);
        const std::vector<Vector3D> &heights = boost::get<3>(result);
        std::cout << "done." << std::endl;

        time_t raw_time;
        struct tm *time_info;
        std::time(&raw_time);
        time_info = localtime(&raw_time);

        char buffer[80];
        strftime(buffer,80,"%d_%m_%H_%I_%M_%S",time_info);
        std::string str(buffer);

        std::string path_to_sensor_data_file = str + "_gen";

        std::stringstream ss;
        ss << data_iter + 1;
        ss << ".txt";
        path_to_sensor_data_file += ss.str();

        std::string path_to_trajectory_file(path_to_sensor_data_file);

        path_to_sensor_data_file = path_to_output_folder_ + "sensor_stream_" + path_to_sensor_data_file;
        path_to_trajectory_file = path_to_output_folder_ + "trajectory_" + path_to_trajectory_file;

        const unsigned int random_seed = simulation.RandomSeed();
        const SystemState system_state = simulation.InitialSystemState();
        const double control_frequency = simulation.ControlFrequency();

        std::cout << "   writing sensor data to file " << path_to_sensor_data_file << " ... ";
        FileWriter writer_sensor_data(path_to_sensor_data_file);
        writer_sensor_data.CreateSensorDataFile(random_seed, control_frequency, data_set_time_, simulation.AsteroidOfSystem(), system_state, labels, data_set);
        std::cout << "done." << std::endl;
        std::cout << "   writing trajectory to file " << path_to_trajectory_file << " ... ";
        FileWriter writer_trajectory(path_to_trajectory_file);
        writer_trajectory.CreateTrajectoryFile(control_frequency, simulation.AsteroidOfSystem(), positions, heights);
        std::cout << "done." << std::endl;
    }
}
