#ifndef FEEDFORWARDNEURALNETWORK_H
#define FEEDFORWARDNEURALNETWORK_H

#include <boost/serialization/access.hpp>

#include <vector>

class FeedForwardNeuralNetwork {
public:
    FeedForwardNeuralNetwork(const unsigned int &dim_input, const unsigned int &dim_hidden, const unsigned int &dim_output);
    ~FeedForwardNeuralNetwork();

    std::vector<double> Evaluate(const std::vector<double> &input);
    void SetWeights(const std::vector<double> &weights);

private:
    friend class boost::serialization::access;

    template <class Archive>

    void serialize(Archive &ar, const unsigned int) {
        ar & const_cast<unsigned int &>(dim_input_);
        ar & const_cast<unsigned int &>(dim_hidden_);
        ar & const_cast<unsigned int &>(dim_output_);
        ar & weights_;
        ar & hidden_;
    }

    unsigned int dim_input_;
    unsigned int dim_hidden_;
    unsigned int dim_output_;

    std::vector<double> weights_;
    std::vector<double> hidden_;
};


#endif // FEEDFORWARDNEURALNETWORK_H
