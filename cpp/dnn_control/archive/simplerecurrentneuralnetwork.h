#ifndef SIMPLERECURRENTNEURALNETWORK_H
#define SIMPLERECURRENTNEURALNETWORK_H

#include "neuralnetwork.h"

class SimpleRecurrentNeuralNetwork : public NeuralNetwork {
public:
    SimpleRecurrentNeuralNetwork(const unsigned int &dim_input, const unsigned int &dim_hidden, const unsigned int &dim_output);
    virtual ~SimpleRecurrentNeuralNetwork();


    virtual std::vector<double> Evaluate(const std::vector<double> &input);

    virtual void SetWeights(const std::vector<double> &weights);

private:
    unsigned int dim_input_;
    unsigned int dim_hidden_;
    unsigned int dim_output_;

    std::vector<double> weights_;
    std::vector<double> hidden_;
    std::vector<double> context_;
};

#endif // SIMPLERECURRENTNEURALNETWORK_H
