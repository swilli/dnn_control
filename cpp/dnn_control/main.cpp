#include "configuration.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationproportionalderivative.h"

#include "filewriter.h"
#include "sensordatagenerator.h"

#include "evolutionaryrobotics.h"

#include "leastsquarespolicyrobotics.h"

int main(int argc, char *argv[]) {
    srand(time(0));

    TrainLeastSquaresPolicyController();

    //SensorDataGenerator generator(PATH_TO_SENSOR_DATA_FOLDER, 86400.0);
    //generator.Generate(1, 0);
}

