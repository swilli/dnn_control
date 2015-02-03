#include "sensordatagenerator.h"
#include "filewriter.h"
#include "pagmosimulationneuralnetwork.h"
#include "samplefactory.h"

#include <ctime>
#include <sstream>

SensorDataGenerator::SensorDataGenerator(const std::string &path_to_output_folder, const double &data_set_time)
    : data_set_time_(data_set_time), path_to_output_folder_(path_to_output_folder) {

}

void SensorDataGenerator::Generate(const unsigned int &num_datasets, const unsigned int &random_seed, const std::string &prefix) {
    SampleFactory sample_factory(random_seed);

    for (unsigned int data_iter = 0; data_iter < num_datasets; ++data_iter) {
        std::cout << "generation " << (data_iter + 1) << " : " << std::endl;
        PaGMOSimulationNeuralNetwork simulation(sample_factory.SampleRandomInteger(), data_set_time_);

        std::cout << "   running simulation ... ";
        const std::vector<SensorData> data_set = simulation.GenerateSensorDataSet();
        std::cout << "done." << std::endl;

        std::string path_to_sensor_data_file(path_to_output_folder_);

        time_t raw_time;
        struct tm *time_info;
        char buffer[80];
        std::time(&raw_time);
        time_info = localtime(&raw_time);

        if (prefix == "") {
            strftime(buffer,80,"data_stream_%d_%m_%H_%I_%M_%S_",time_info);
        } else{
            path_to_sensor_data_file += prefix;
            strftime(buffer,80,"_%d_%m_%H_%I_%M_%S_",time_info);

        }

        std::string str(buffer);
        path_to_sensor_data_file += str;

        path_to_sensor_data_file += "gen";
        std::stringstream ss;
        ss << data_iter + 1;
        ss << ".txt";
        path_to_sensor_data_file += ss.str();


        const unsigned int random_seed = simulation.RandomSeed();
        const SystemState system_state = simulation.InitialSystemState();

        std::cout << "   writing sensor data to file " << path_to_sensor_data_file << " ... ";
        FileWriter writer;
        writer.CreateSensorDataFile(path_to_sensor_data_file, random_seed, simulation.InteractionInterval(), simulation.SimulationTime(), simulation.AsteroidOfSystem(),
                                    system_state, data_set);
        std::cout << "done." << std::endl;
    }
}
