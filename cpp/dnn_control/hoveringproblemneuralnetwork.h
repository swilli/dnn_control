#ifndef HOVERINGPROBLEMNEURALNETWORK_H
#define HOVERINGPROBLEMNEURALNETWORK_H

#include "pagmosimulationneuralnetwork.h"

#include <pagmo/src/problem/base_stochastic.h>
#include <boost/serialization/access.hpp>

namespace pagmo { namespace problem {

class __PAGMO_VISIBLE hovering_problem_neural_network : public base_stochastic {
    /*
    * This class represents a PaGMO problem which can be optimized using a base stochastic algorithm. 
    * The task in this problem is to either hover over a specific target position, or to keep a certain height above the asteroid.
    */
public:
    hovering_problem_neural_network(const unsigned int &seed=0, const unsigned int &n_evaluations=4, const double &simulation_time=3600.0, const unsigned int &n_hidden_neurons=6);

    hovering_problem_neural_network(const hovering_problem_neural_network &other);

    ~hovering_problem_neural_network();

    // Perform multiple evaluations with a solution on the problem
    boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluate(const decision_vector &x, const unsigned int &start_seed=0, const std::vector<unsigned int> &random_seeds=std::vector<unsigned int>()) const;

    // Returns the problem name
    std::string get_name() const;

    // Necessary for polymorphism
    base_ptr clone() const;

    // Returns the fitness of a solution with respect to a seeded (= deterministic problem) simulation
    fitness_vector objfun_seeded(const unsigned int &seed, const decision_vector &x) const;

protected:
    void objfun_impl(fitness_vector &f, const decision_vector &x) const;

    std::string human_readable_extra() const;

private:
    // Implementation of the problems fitness
    double single_fitness(PaGMOSimulationNeuralNetwork &simulation) const;

    // Returns more information about the problem
    double single_post_evaluation(PaGMOSimulationNeuralNetwork &simulation) const;

    // Number of evaluations an indiviual will be tested in a generation
    unsigned int m_n_evaluations;

    // Number of hidden nodes in the controller's Neural Network
    unsigned int m_n_hidden_neurons;

    // Time one simulation will be run
    double m_simulation_time;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int) {
        ar & boost::serialization::base_object<base_stochastic>(*this);
        ar & m_n_evaluations;
        ar & m_n_hidden_neurons;
        ar & m_simulation_time;
    }
};

}} //namespaces

BOOST_CLASS_EXPORT_KEY(pagmo::problem::hovering_problem_neural_network)

#endif // HOVERINGPROBLEMNEURALNETWORK_H
