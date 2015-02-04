#ifndef EVOLUTIONARYROBOTICS_H
#define EVOLUTIONARYROBOTICS_H

#include <pagmo/src/pagmo.h>

void TrainNeuralNetworkController();

void TestNeuralNetworkController(const pagmo::decision_vector &controller_parameters);

#endif // EVOLUTIONARYROBOTICS_H
