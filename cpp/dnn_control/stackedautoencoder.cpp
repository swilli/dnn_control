#include "stackedautoencoder.h"
#include "configuration.h"

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <sstream>
#include <regex>

StackedAutoencoder::StackedAutoencoder(const std::string &path_to_layer_configurations) {
    using namespace boost::filesystem;

    const unsigned int num_files_per_compression_layer = 4;

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

    std::regex regex_compression(".*/al.*\.txt");
    std::regex regex_supervised(".*/sl.*\.txt");

    std::vector<std::string> compression_files;
    std::vector<std::string> supervised_files;
    std::string configuration_file;

    for (unsigned int i = 0; i < file_paths.size(); ++i) {
        const std::string &fpath = file_paths.at(i);
        if (fpath.find("/conf.txt") != std::string::npos) {
            configuration_file = fpath;
        } else if (std::regex_match(fpath, regex_compression)) {
            compression_files.push_back(fpath);
        } else if (std::regex_match(fpath, regex_supervised)) {
            supervised_files.push_back(fpath);
        }
    }

    const boost::tuple<unsigned int, unsigned int, bool, std::vector<boost::tuple<unsigned int, bool> > > configuration = ParseConfigFile(configuration_file);
    const unsigned int &input_size = boost::get<0>(configuration);
    const unsigned int &output_size = boost::get<1>(configuration);
    const bool &supervised_sigmoid_activation = boost::get<2>(configuration);
    const std::vector<boost::tuple<unsigned int, bool> > &compression_layer_configurations = boost::get<3>(configuration);

    std::vector<boost::tuple<unsigned int, bool, NeuralNetwork::ActivationFunctionType> > layer_configurations;
    std::vector<double> network_weights;

    unsigned int dim_layer_input = input_size;
    unsigned int dim_layer_output = input_size;
    for (unsigned int i = 0; i < compression_layer_configurations.size(); ++i) {
        const boost::tuple<unsigned int, bool> &clconf = compression_layer_configurations.at(i);
        NeuralNetwork::ActivationFunctionType fun_type = NeuralNetwork::ActivationFunctionType::Linear;
        if (boost::get<1>(clconf)) {
            fun_type = NeuralNetwork::ActivationFunctionType::Sigmoid;
        }
        dim_layer_output = boost::get<0>(clconf);
        layer_configurations.push_back(boost::make_tuple(dim_layer_output, true, fun_type));

        const std::vector<std::vector<double> > weights = ParseWeightMatrix(compression_files.at(num_files_per_compression_layer*i));
        const std::vector<double> bias = ParseBiasVector(compression_files.at(num_files_per_compression_layer*i + 2));

        if (bias.size() != dim_layer_output || weights.size() != dim_layer_output || weights.at(0).size() != dim_layer_input) {
            throw ConfigurationMismatch();
        }
        for (unsigned int j = 0; j < dim_layer_output; ++j) {
            network_weights.push_back(bias.at(j));
            for (unsigned int k = 0; k < dim_layer_input; ++k) {
                network_weights.push_back(weights.at(j).at(k));
            }
        }

        dim_layer_input = dim_layer_output;
    }

    dim_layer_output = output_size;
    NeuralNetwork::ActivationFunctionType fun_type = NeuralNetwork::ActivationFunctionType::Linear;
    if (supervised_sigmoid_activation) {
        fun_type = NeuralNetwork::ActivationFunctionType::Sigmoid;
    }

    const std::vector<std::vector<double> > weights = ParseWeightMatrix(supervised_files.at(0));
    const std::vector<double> bias = ParseBiasVector(supervised_files.at(1));

    if (bias.size() != dim_layer_output || weights.size() != dim_layer_output || weights.at(0).size() != dim_layer_input) {
        throw ConfigurationMismatch();
    }

    for (unsigned int j = 0; j < dim_layer_output; ++j) {
        network_weights.push_back(bias.at(j));
        for (unsigned int k = 0; k < dim_layer_input; ++k) {
            network_weights.push_back(weights.at(j).at(k));
        }
    }

    neural_network_ = FeedForwardNeuralNetwork(input_size, true, output_size, fun_type, layer_configurations);
    neural_network_.SetWeights(network_weights);
}

std::vector<double> StackedAutoencoder::Evaluate(const std::vector<double> &input) {
    return neural_network_.Evaluate(input);
}

unsigned int StackedAutoencoder::InputDimension() const {
    return neural_network_.InputDimension();
}

unsigned int StackedAutoencoder::OutputDimension() const {
    return neural_network_.OutputDimension();
}

std::vector<std::vector<double> > StackedAutoencoder::ParseWeightMatrix(const std::string &path_to_matrix) {
    std::ifstream reader;
    reader.open(path_to_matrix);
    std::vector<std::string> file_data_string;
    std::string buffer;
    while (std::getline(reader, buffer)) {
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
            std::getline(ss, substr, ',');
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
        std::getline(ss, substr, ',');
        file_data_values.push_back(substr);
    }

    std::vector<double> bias_weights;
    std::transform(file_data_values.begin(), file_data_values.end(), std::back_inserter(bias_weights), [](const std::string &astr)
    {
        return std::stod(astr);
    }) ;

    return bias_weights;
}

boost::tuple<unsigned int, unsigned int, bool, std::vector<boost::tuple<unsigned int, bool> > > StackedAutoencoder::ParseConfigFile(const std::string &path_to_config) {
    std::ifstream reader;
    reader.open(path_to_config);
    std::vector<std::string> file_data_lines;
    std::string buffer;
    while (std::getline(reader, buffer)) {
        file_data_lines.push_back(buffer);
    }
    reader.close();

    std::vector<std::string> tmp_strings;
    std::stringstream ss(file_data_lines.at(0));
    while (ss.good()) {
        std::string substr;
        std::getline(ss, substr, ',');
        substr.erase(std::remove_if(substr.begin(), substr.end(), ::isspace), substr.end());
        tmp_strings.push_back(substr);
    }

    const unsigned int dim_input = std::stoi(tmp_strings.at(0));
    const unsigned int dim_output = std::stoi(tmp_strings.at(1));
    const bool supervised_layer_sigmoid_activation = tmp_strings.at(2) == "True";

    std::vector<boost::tuple<unsigned int, bool> > layer_configs;

    std::stringstream ss_hidden_layer_sizes(file_data_lines.at(1));
    std::stringstream ss_sigmoid_compression(file_data_lines.at(3));

    while (ss_hidden_layer_sizes.good() && ss_sigmoid_compression.good()) {
        std::string str_size;
        std::string str_compr;

        std::getline(ss_hidden_layer_sizes, str_size, ',');
        std::getline(ss_sigmoid_compression, str_compr, ',');

        str_size.erase(std::remove_if(str_size.begin(), str_size.end(), ::isspace), str_size.end());
        str_compr.erase(std::remove_if(str_compr.begin(), str_compr.end(), ::isspace), str_compr.end());

        if (str_size.length() == 0 || str_compr.size() == 0) {
            break;
        }
        const unsigned int size = std::stoi(str_size);
        const bool sigmoid_compression = str_compr == "True";

        layer_configs.push_back(boost::make_tuple(size, sigmoid_compression));
    }

    return boost::make_tuple(dim_input, dim_output, supervised_layer_sigmoid_activation, layer_configs);
}
