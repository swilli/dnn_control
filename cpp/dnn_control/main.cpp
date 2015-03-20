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


    /*
    std::ifstream reader;
    reader.open("/home/willist/Documents/dnn/results/encoder_test.text");
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
            std::getline(ss, substr, ',' );
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
        const std::vector<double> output_cpp = sda.Compress(input);
        double norm = 0.0;
        for (unsigned int j = 0; j < correct_output.size(); ++j) {
            norm += (correct_output.at(j) - output_cpp.at(j)) * (correct_output.at(j) - output_cpp.at(j));
        }
        norm = sqrt(norm);
        error += norm;
    }
    std::cout << error/num_tests << std::endl;
    return 0;
    */

    //TestNeuralNetworkController(0);
    //return 0;

    //TestProportionalDerivativeController(0);
    //return 0;

    //TestNeuralNetworkVSFullStateController(9782);
    //return 0;

    //TestLeastSquaresPolicyController(9782);
    //return 0;

    //TrainNeuralNetworkController();
    //return 0;

    //TrainProportionalDerivativeController();
    // return 0;

    //TrainLeastSquaresPolicyController();
    //return 0;



    SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 6.0 * 60.0 * 60.0);
    const std::vector<double> solution = {5.602494788, -3.371097445, 0.01746151859, -2.465648547, 0.2487294463, -6.420506703, 6.494730276, -0.1678714229, 9.688968003, 8.048570769, -0.8928683344, 23.80353092, 3.787328341, 0.4481259054, -0.1109844818, 0.6590247967, 6.304021438, -3.281716374, -1.143023643, 16.77765445, -3.492812899, 0.02490513228, 10.3777212, -9.71059683, -7.099271347, 1.564616455, -13.71976281, -15.62627194, -0.2751338119, -10.13537359, 1.204235381, 1.81071888, -5.498129643, 12.20552779, 5.482914803, 0.1386384164, -7.295089538, 8.643225194, -9.46373845, -6.005790683, -2.178750444, -13.55225592, 2.760527294, -3.75662734, 14.1459134, 2.95190773, 3.502740882, -13.1779763, -5.635020081, -3.412016341, -6.764106964, 5.924421528, 16.83398466, -9.27800504, 2.826960963, 5.785092178, 4.414910215, 3.753662375, 2.66269484, -4.817543969, -7.717323264, 5.952869873, -10.75391098};
    generator.Generate(1000, rand(), solution);
    return 0;

}

