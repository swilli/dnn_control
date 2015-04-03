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

    //TestLeastSquaresPolicyController(0);
    //return 0;
    //TrainLeastSquaresPolicyController();
    //return 0;

    SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 300.0);
    generator.Generate(15000, 0);
}

