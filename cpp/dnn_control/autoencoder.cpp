#include "autoencoder.h"

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <sstream>

Autoencoder::Autoencoder(const std::string &path_to_layer_configurations) {
    using namespace boost::filesystem;

    path configuration_dir(path_to_layer_configurations);

    directory_iterator end_itr;

    std::vector<std::string> file_paths;
    for (directory_iterator itr(configuration_dir); itr != end_itr; ++itr)
    {
        if (is_regular_file(itr->path())) {
            const std::string current_file = itr->path().string();
            file_paths.push_back(current_file);
        }
    }

    std::sort(file_paths.begin(), file_paths.end());

    unsigned int dim_input = 0;
    const unsigned int num_hidden_layers = file_paths.size() / 3;

    std::vector<boost::tuple<unsigned int, bool, NeuralNetwork::ActivationFunctionType> > layer_configurations;
    std::vector<double> encoder_weights;
    for (unsigned int i = 0; i < num_hidden_layers; ++i) {
        const std::vector<std::vector<double> > weights = ParseWeightMatrix(file_paths.at(3*i));
        const std::vector<double> bias = ParseBiasVector(file_paths.at(3*i + 1));

        const unsigned int dim_layer_input = weights.at(0).size();
        if (i == 0) {
            dim_input = dim_layer_input;
        }
        const unsigned int dim_layer_output = weights.size();
        layer_configurations.push_back(boost::make_tuple(dim_layer_output, true, NeuralNetwork::ActivationFunctionType::Sigmoid));
        for (unsigned int i = 0; i < dim_layer_output; ++i) {
            encoder_weights.push_back(bias.at(i));
            for (unsigned int j = 0; j < dim_layer_input; ++j) {
                encoder_weights.push_back(weights.at(i).at(j));
            }
        }
    }

    neural_network_ = FeedForwardNeuralNetwork(dim_input, true, layer_configurations);
    neural_network_.SetWeights(encoder_weights);
}

std::vector<double> Autoencoder::Compress(const std::vector<double> &input) {
    return neural_network_.Evaluate(input);
}

std::vector<std::vector<double> > Autoencoder::ParseWeightMatrix(const std::string &path_to_matrix) {
    std::ifstream reader;
    reader.open(path_to_matrix);
    std::vector<std::string> file_data_string;
    std::string buffer;
    while(std::getline(reader, buffer)) {
        file_data_string.push_back(buffer);
    }
    reader.close();

    std::vector<std::vector<double> > weights;
    for (unsigned int i = 0; i < file_data_string.size(); ++i) {
        std::vector<std::string> tmp_strings;
        std::vector<double> tmp_doubles;
        std::stringstream ss(file_data_string.at(i));
        while (ss.good()) {
            std::string substr;
            std::getline(ss, substr, ',' );
            tmp_strings.push_back(substr);
        }
        std::transform(tmp_strings.begin(), tmp_strings.end(), std::back_inserter(tmp_doubles), [](const std::string &astr)
        {
            return std::stod(astr);
        }) ;
        weights.push_back(tmp_doubles);
    }
    return weights;
}

std::vector<double> Autoencoder::ParseBiasVector(const std::string &path_to_vector) {
    std::ifstream reader;
    reader.open(path_to_vector);
    std::string file_data_string;
    std::getline(reader, file_data_string);
    reader.close();

    std::vector<std::string> file_data_values;
    std::stringstream ss(file_data_string);
    while (ss.good()) {
        std::string substr;
        std::getline(ss, substr, ',' );
        file_data_values.push_back(substr);
    }

    std::vector<double> bias_weights;
    std::transform(file_data_values.begin(), file_data_values.end(), std::back_inserter(bias_weights), [](const std::string &astr)
    {
        return std::stod(astr);
    }) ;

    return bias_weights;
}
