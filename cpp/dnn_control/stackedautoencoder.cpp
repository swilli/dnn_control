#include "stackedautoencoder.h"
#include "configuration.h"

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <sstream>

StackedAutoencoder::StackedAutoencoder(const std::string &path_to_layer_configurations) {
    using namespace boost::filesystem;

    input_size_ = 0;
    output_size_ = 0;

#if CNN_ENABLE_STACKED_AUTOENCODER == false
    return;
#endif

    const unsigned int num_files_per_layer = 4;
    const unsigned int num_supervised_layer_files = 2;

    path configuration_dir(path_to_layer_configurations);
    if (!exists(configuration_dir)) {
        throw ConfigurationFolderDoesNotExist();
    }

    directory_iterator end_itr;

    std::vector<std::string> file_paths;
    for (directory_iterator itr(configuration_dir); itr != end_itr; ++itr)
    {
        if (is_regular_file(itr->path())) {
            const std::string current_file = itr->path().string();
            file_paths.push_back(current_file);
        }
    }

    if (file_paths.size() == 0) {
        return;
    }

    std::sort(file_paths.begin(), file_paths.end());

    input_size_ = 0;
    output_size_ = 0;

    const unsigned int num_hidden_layers = (file_paths.size() - num_supervised_layer_files) / num_files_per_layer;

    std::vector<boost::tuple<unsigned int, bool, NeuralNetwork::ActivationFunctionType> > layer_configurations;
    std::vector<double> encoder_weights;
    for (unsigned int i = 0; i < num_hidden_layers; ++i) {
        const std::vector<std::vector<double> > weights = ParseWeightMatrix(file_paths.at(num_files_per_layer*i));
        const std::vector<double> bias = ParseBiasVector(file_paths.at(num_files_per_layer*i + 2));

        const unsigned int dim_layer_input = weights.at(0).size();
        const unsigned int dim_layer_output = weights.size();
        if (i == 0) {
            input_size_ = dim_layer_input;
        }
        if (i == (num_hidden_layers - 1)) {
            output_size_ = dim_layer_output;
        } else {
            layer_configurations.push_back(boost::make_tuple(dim_layer_output, true, NeuralNetwork::ActivationFunctionType::Sigmoid));
        }
        for (unsigned int i = 0; i < dim_layer_output; ++i) {
            encoder_weights.push_back(bias.at(i));
            for (unsigned int j = 0; j < dim_layer_input; ++j) {
                encoder_weights.push_back(weights.at(i).at(j));
            }
        }
    }

    neural_network_ = FeedForwardNeuralNetwork(input_size_, true, output_size_, NeuralNetwork::ActivationFunctionType::Sigmoid, layer_configurations);
    neural_network_.SetWeights(encoder_weights);
}

std::vector<double> StackedAutoencoder::Compress(const std::vector<double> &input) {
    return neural_network_.Evaluate(input);
}

unsigned int StackedAutoencoder::InputSize() const {
    return input_size_;
}

unsigned int StackedAutoencoder::OutputSize() const {
    return output_size_;
}

std::vector<std::vector<double> > StackedAutoencoder::ParseWeightMatrix(const std::string &path_to_matrix) {
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

std::vector<double> StackedAutoencoder::ParseBiasVector(const std::string &path_to_vector) {
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
