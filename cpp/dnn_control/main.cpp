#include "configuration.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationproportionalderivative.h"

#include "filewriter.h"
#include "sensordatagenerator.h"

#include "evolutionaryrobotics.h"

#include "leastsquarespolicyrobotics.h"

#include "stackedautoencoder.h"

int main(int argc, char *argv[]) {
    srand(time(0));

    std::ifstream reader;
    reader.open("/home/willist/Documents/dnn/results/implementation.txt");
    std::vector<std::string> file_data_string;
    std::string buffer;
    while(std::getline(reader, buffer)) {
        file_data_string.push_back(buffer);
    }
    reader.close();

    std::vector<std::vector<double> > values;
    for (unsigned int i = 0; i < file_data_string.size(); ++i) {
        std::vector<std::string> tmp_strings;
        std::vector<double> tmp_doubles;
        std::stringstream ss(file_data_string.at(i));
        while (ss.good()) {
            std::string substr;
            std::getline(ss, substr, ',');
            tmp_strings.push_back(substr);
        }
        std::transform(tmp_strings.begin(), tmp_strings.end(), std::back_inserter(tmp_doubles), [](const std::string &astr)
        {
            return std::stod(astr);
        }) ;
        values.push_back(tmp_doubles);
    }

    StackedAutoencoder sda(PATH_TO_AUTOENCODER_LAYER_CONFIGURATION);

    const unsigned int num_tests = values.size()/2;
    double error = 0.0;

    for (unsigned int i = 0; i < num_tests; ++i) {
        const std::vector<double> &input = values.at(2*i);
        const std::vector<double> &correct_output = values.at(2*i + 1);
        const std::vector<double> output_cpp = sda.Evaluate(input);
        double norm = 0.0;
        for (unsigned int j = 0; j < correct_output.size(); ++j) {
            norm += (correct_output.at(j) - output_cpp.at(j)) * (correct_output.at(j) - output_cpp.at(j));
        }
        norm = sqrt(norm);
        error += norm;
    }
    std::cout << error/num_tests << std::endl;

    //TrainNeuralNetworkController();
    //return 0;

    //SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 300.0);
    //generator.Generate(15000, 0);
}

