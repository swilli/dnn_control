#include "sensordatagenerator.h"
#include "evolutionaryrobotics.h"
#include "leastsquarespolicyrobotics.h"


int main(int argc, char *argv[]) {
    srand(time(0));

    //TrainNeuralNetworkController();
    TestNeuralNetworkController(0);
    return 0;
}

