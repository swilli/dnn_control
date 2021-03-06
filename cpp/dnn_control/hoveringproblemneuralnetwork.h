#ifndef HOVERINGPROBLEMNEURALNETWORK_H
#define HOVERINGPROBLEMNEURALNETWORK_H

#include "pagmosimulationneuralnetwork.h"
#include "sensorsimulator.h"

#include <pagmo/src/problem/base_stochastic.h>
#include <boost/serialization/access.hpp>

namespace pagmo { namespace problem {

class __PAGMO_VISIBLE hovering_problem_neural_network : public base_stochastic {
    /*
    * This class represents a PaGMO problem which can be optimized using a base stochastic algorithm. 
    * The optimization problem is to find a neural network controller which minimizes the objective function.
    */
public:

    // The different types of fitness functions
    enum FitnessFunctionType {
        FitnessCompareStartEndPosition,
        FitnessAveragePositionOffset,
        FitnessAveragePositionOffsetAndVelocity,
        FitnessAveragePositionOffsetAndFuel,
        FitnessAveragePositionOffsetAndVelocityAndFuel,
        FitnessAverageVelocity,
        FitnessAverageOpticFlowAndConstantDivergence
    };

    // The different types of post evaluation functions
    enum PostEvaluationFunctionType {
        PostEvalAveragePositionOffset,
        PostEvalAverageVelocity
    };

    hovering_problem_neural_network(const unsigned int &seed=0, const unsigned int &n_evaluations=10, const double &simulation_time=0.0, const unsigned int &n_hidden_neurons=6, const std::set<SensorSimulator::SensorType> &sensor_types={}, const bool &enable_sensor_noise=false,
                                    const FitnessFunctionType &fitness_function=FitnessFunctionType::FitnessAveragePositionOffsetAndVelocity, const PostEvaluationFunctionType &post_evaluation_function=PostEvaluationFunctionType::PostEvalAveragePositionOffset,
                                    const double &transient_response_time=150.0, const double &divergence_set_value=1e-3);

    hovering_problem_neural_network(const hovering_problem_neural_network &other);


    // Perform multiple evaluations with a solution on the problem, returns the seeds used, the mean, min and max error for each simulation.
    boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> >, std::vector<std::pair<double, double > > > post_evaluate(const decision_vector &x, const unsigned int &start_seed=0, const std::vector<unsigned int> &random_seeds=std::vector<unsigned int>()) const;

    // Returns the problem name
    std::string get_name() const;

    // Necessary for polymorphism
    base_ptr clone() const;

    // Returns the fitness of a solution with respect to a seeded (= deterministic problem) simulation
    fitness_vector objfun_seeded(const unsigned int &seed, const decision_vector &x) const;

    // HoveringProblemNeuralNetwork can throw the following exceptions
    class Exception {};
    class FitnessFunctionTypeNotImplemented : public Exception {};
    class PostEvaluationFunctionTypeNotImplemented : public Exception {};

protected:
    void objfun_impl(fitness_vector &f, const decision_vector &x) const;

    std::string human_readable_extra() const;

private:
    // Implementation of the problems fitness
    double single_fitness(PaGMOSimulationNeuralNetwork &simulation) const;

    // Performs the simulation, computes the mean, min, max error based on the generated data.
    // Error can be different from fitness.
    boost::tuple<double, double, double, double, double> single_post_evaluation(PaGMOSimulationNeuralNetwork &simulation) const;

    // Number of evaluations an indiviual will be tested in a generation
    unsigned int m_n_evaluations;

    // Number of hidden nodes in the controller's Neural Network
    unsigned int m_n_hidden_neurons;

    // Time one simulation will be run
    double m_simulation_time;

    // The control sensor types
    std::set<SensorSimulator::SensorType> m_sensor_types;

    // Is noise enabled in the sensor simulator
    bool m_enable_sensor_noise;

    // The fitness function type
    enum FitnessFunctionType m_fitness_function;

    // The post evaluation type
    enum PostEvaluationFunctionType m_post_evaluation_function;

    // The transient response, if needed
    double m_transient_response_time;

    // The set divergence value, if needed
    double m_divergence_set_value;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int) {
        ar & boost::serialization::base_object<base_stochastic>(*this);
        ar & m_n_evaluations;
        ar & m_n_hidden_neurons;
        ar & m_simulation_time;
        ar & m_sensor_types;
        ar & m_enable_sensor_noise;
        ar & m_fitness_function;
        ar & m_post_evaluation_function;
        ar & m_transient_response_time;
        ar & m_divergence_set_value;
    }
};

}} //namespaces

BOOST_CLASS_EXPORT_KEY(pagmo::problem::hovering_problem_neural_network)

#endif // HOVERINGPROBLEMNEURALNETWORK_H
