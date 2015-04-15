#include "sensordatagenerator.h"
#include "evolutionaryrobotics.h"
#include "leastsquarespolicyrobotics.h"


int main(int argc, char *argv[]) {
    srand(time(0));

    TrainLeastSquaresPolicyController();
    return 0;
}

