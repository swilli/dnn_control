#include "configuration.h"
#include "pagmosimulationneuralnetwork.h"
#include "pagmosimulationproportionalderivative.h"

#include "filewriter.h"
#include "sensordatagenerator.h"

#include "evolutionaryrobotics.h"

#include "leastsquarespolicyrobotics.h"

int main(int argc, char *argv[]) {
    srand(time(0));

    const unsigned int num_samples = 100000;
    for (unsigned int i =0; i < num_samples; ++i) {
        const unsigned int seed= rand();
        PaGMOSimulationNeuralNetwork(seed, 3600);
    }
    return 0;
}

