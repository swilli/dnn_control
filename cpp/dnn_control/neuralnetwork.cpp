#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork() {
    size_ = 0;
}

NeuralNetwork::~NeuralNetwork() {

}

unsigned int NeuralNetwork::Size() const {
    return size_;
}
