#include "configuration.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationproportionalderivative.h"

#include "filewriter.h"
#include "sensordatagenerator.h"

#include "evolutionaryrobotics.h"

#include "leastsquarespolicyrobotics.h"

int main(int argc, char *argv[]) {
    srand(time(0));

    TestNeuralNetworkController(0);
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
    //return 0;

    //TrainLeastSquaresPolicyController();
    //return 0;

    SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 6.0 * 60.0 * 60.0);
    const std::vector<double> solution = {-0.09026912926, 8.138242141, 1.179279962, 0.9967504316, 9.107872994, 2.495094098, 1.21491874, 1.075098664, 6.478080265, -1.760053375, -5.557077224, 0.07087069288, -3.857047326, -1.142817024, -0.04177031103, -5.545367675, 6.897913933, -0.868805931, -5.581981788, 6.81000191, -0.6537881697, -3.01382613, -3.460458177, -1.222132272, 6.661425517, -0.8429447432, 1.847847975, -0.4622668776, 0.2102273386, 4.604822609, 6.7988227, -1.35256986, 3.409805751, 5.066750139, -2.000987524, 0.2158515067, -1.844968104, 2.081369052, 7.031798061, -3.251052176, 3.378869104, 7.205035636, 1.779003246, -4.6738817, -1.964513429, 3.136008676, -0.5397410952, -3.366847512, 4.220415052, 0.05860299037, 0.9929627838, 2.300526511, -4.63787856, 1.556752539, -3.874841742, 3.831621996, 3.293583417, -1.845062421, 1.584113831, 0.6596747896, 0.06139593008, -2.663128882, -4.332896525};


    generator.Generate(1, rand(), solution);
    return 0;
}

