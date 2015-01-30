#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork(const unsigned int &size)
    : size_(size) {

}

NeuralNetwork::~NeuralNetwork()
{

}

unsigned int NeuralNetwork::Size() const {
    return size_;
}

