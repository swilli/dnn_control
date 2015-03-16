#include "configuration.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationproportionalderivative.h"

#include "filewriter.h"
#include "sensordatagenerator.h"

#include "evolutionaryrobotics.h"

#include "leastsquarespolicyrobotics.h"

int main(int argc, char *argv[]) {
    srand(time(0));

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
    //return 0;

    //TrainLeastSquaresPolicyController();
    //return 0;

    SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 6.0 * 60.0 * 60.0);
    const std::vector<double> solution = {-0.2837463395, 8.859739303, 0.01162629475, 0.671527106, 7.893611325, 1.554894924, 2.312587263, 0.5334521505, 6.092724937, -0.2753838158, -3.229783156, 2.065080307, -4.037341544, 0.2375259085, -0.3063833758, -4.763986291, 6.550462251, -0.00787201929, -5.153368593, 7.168039971, -0.4308486251, -3.722991167, -1.947515387, -1.389036352, 4.680722995, 0.3905330517, 1.578777557, 1.625963363, 0.1043750019, 5.589552232, 5.873822623, -1.404802157, 4.149557415, 6.539724984, -1.856184441, 0.1474507903, -1.619575793, 3.011105111, 10.34629026, -3.205443639, 2.56097349, 11.35385515, 1.573362515, -4.86131507, -1.643941525, 3.216527618, -0.4279798467, -3.769955541, 4.302508066, 0.04991456034, 0.6086825301, 2.538886724, -4.123438091, 1.998919629, -4.128025201, 3.564443658, 3.218125812, -1.898824285, 1.529901831, 0.9448832609, 0.247584855, -2.893534827, -4.214792224};


    generator.Generate(1, rand(), solution);
    return 0;
}

