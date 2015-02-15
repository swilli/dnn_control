#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <vector>

class NeuralNetwork {
    /*
    * This abstract class represents a Neural Network. 
    */
public:
    NeuralNetwork();
    virtual ~NeuralNetwork();

    // Evaluate input data by a forward pass through the network
    virtual std::vector<double> Evaluate(const std::vector<double> &input) = 0;

    // Change the FFNN output by changing its weights
    virtual void SetWeights(const std::vector<double> &weights) = 0;

    // Returns the total amount of weights in the neural network
    virtual unsigned int Size() const;

    // NeuralNetwork can throw the following exceptions
    class Exception {};
    class SizeMismatchException : public Exception {};

    // Possible activation functions in a layer
    enum ActivationFunctionType {
        Linear,
        Sigmoid
    };

protected:
    // The total amount of weights in the neural network
    unsigned int size_;
};

#endif // NEURALNETWORK_H
