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
    TestLeastSquaresPolicyController(0);
}

