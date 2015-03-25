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

    SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 60.0);
    const std::vector<double> solution = {7.469570848, -7.471039786, -1.380818276, -1.401772261, -5.452932277, -10.27438984, 11.53858831, 0.122629733, 13.16368449, 8.767684724, 1.556950963, 28.0178391, 2.76982292, 5.62367377, -0.260902508, 3.518733981, 8.495503044, -2.064526712, -1.293223812, 28.69656108, -8.797957997, -0.1685736009, 3.969903076, -17.51792035, -13.86604479, -2.202591289, -12.43105627, -20.12988731, -0.3628840097, -20.05911985, -2.57923473, 9.323881117, -1.766343366, 22.00987951, 8.36918794, 0.07921703441, -7.429925438, 9.919820816, -14.91835189, -4.818657707, 0.8748167487, -7.52185307, 2.360419274, -3.708359255, 16.90756987, 5.682871306, 2.504824962, -16.37004989, -8.731155083, -3.31352264, -5.938812505, 6.045723878, 17.75955573, -12.01433638, -0.06788037391, 7.38388616, 5.723640879, 4.695731869, 4.615659602, -6.679488086, -10.20787269, 7.535243253, -16.08736865};
    generator.Generate(15000, rand(), solution);
    return 0;
}

