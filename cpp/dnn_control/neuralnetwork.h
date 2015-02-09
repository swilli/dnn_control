#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <vector>

class NeuralNetwork {
public:
    NeuralNetwork(const unsigned int &size);
    virtual ~NeuralNetwork();

    virtual std::vector<double> Evaluate(const std::vector<double> &input) = 0;

    virtual void SetWeights(const std::vector<double> &weights) = 0;

    unsigned int Size() const;

    static unsigned int TotalSizeForNetworkConfiguration(const std::vector<std::pair<unsigned int, bool> > &layer_configurations);

    class Exception {};
    class SizeMismatchException : public Exception {};

protected:
    unsigned int size_;
};

#endif // NEURALNETWORK_H
