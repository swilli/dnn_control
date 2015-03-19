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

    TrainNeuralNetworkController();
    return 0;

    //TrainProportionalDerivativeController();
    // return 0;

    //TrainLeastSquaresPolicyController();
    //return 0;

    /*
    SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 6.0 * 60.0 * 60.0);
    const std::vector<double> solution = {0.1257435762, -4.393217578, -5.088278152, 12.79457528, -9.619560236, 1.84323572, 0.9043928026, 0.03307936687, 0.005108165992, -3.079204776, -5.483935086, -7.18401306, -2.685465983, -17.52764864, 2.484939975, -2.365873338, 0.4437299199, 3.091839921, -4.672508584, -1.7029324, 5.239970477, -0.2957594836, -19.92990522, 0.6478485068, -7.748456307, -3.573637709, -2.209799145, -4.179988454, -0.4883468614, 8.308186355, 0.8751278989, 0.6421287721, 16.73051772, 1.993104343, -1.264191833, -0.1845810096, 0.5397372113, 9.930376056, 0.8992046511, -7.488057466, 15.61170728, -3.235207517, 2.04491186, -3.743253854, 0.9911108317, -0.7995093032, -4.228119263, 8.002738991, -2.302391543, -0.8473874197, -1.816443013, -4.432361724, -1.413361716, 1.052831653, -2.229837337, 12.67655027, -0.8303756074, 4.057023863, -8.240327338, 4.496694629, -1.122371878, -1.507073069, -0.5052202112};
    generator.Generate(10000, rand(), solution);
    return 0;
    */
}

