#ifndef EVOLUTIONARYROBOTICS_H
#define EVOLUTIONARYROBOTICS_H

#include <pagmo/src/pagmo.h>

void TrainNeuralNetworkController();

void TestNeuralNetworkController(const pagmo::decision_vector &champion);

#endif // EVOLUTIONARYROBOTICS_H
