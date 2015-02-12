#ifndef EVOLUTIONARYROBOTICS_H
#define EVOLUTIONARYROBOTICS_H

#include <pagmo/src/pagmo.h>

void TrainNeuralNetworkController();
void TestNeuralNetworkController(const unsigned int &random_seed);

void TrainFullStateController();
void TestFullStateController(const unsigned int &random_seed);

#endif // EVOLUTIONARYROBOTICS_H
