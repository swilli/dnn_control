#ifndef HOVERINGPROBLEMPROPORTIONALDERIVATIVE_H
#define HOVERINGPROBLEMPROPORTIONALDERIVATIVE_H

#include "pagmosimulationproportionalderivative.h"

#include <pagmo/src/problem/base_stochastic.h>
#include <boost/serialization/access.hpp>

namespace pagmo { namespace problem {

class __PAGMO_VISIBLE hovering_problem_proportional_derivative : public base_stochastic {
    /*
    * This class represents a PaGMO problem which can be optimized using a base stochastic algorithm. 
    * The optimization problem is to find a proportional derivative controller which minimizes the objective function.
    */
public:
    hovering_problem_proportional_derivative(const unsigned int &seed=0, const unsigned int &n_evaluations=4, const double &simulation_time=3600.0);

    hovering_problem_proportional_derivative(const hovering_problem_proportional_derivative &other);

    ~hovering_problem_proportional_derivative();

    // Perform multiple evaluations with a solution on the problem, returns the seeds used, the mean, min and max error for each simulation.
    boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> > > post_evaluate(const decision_vector &x, const unsigned int &start_seed=0, const std::vector<unsigned int> &random_seeds=std::vector<unsigned int>()) const;

    // Returns the problem name
    std::string get_name() const;

    // Necessary for polymorphism
    base_ptr clone() const;

    // Returns the fitness of a solution with respect to a seeded (= deterministic problem) simulation
    fitness_vector objfun_seeded(const unsigned int &seed, const decision_vector &x) const;

protected:
    // Implementation of the problems fitness
    void objfun_impl(fitness_vector &f, const decision_vector &x) const;

    // Returns more information about the problem
    std::string human_readable_extra() const;

private:
    // Performs the simulation, computes the fitness based on the generated data
    double single_fitness(PaGMOSimulationProportionalDerivative &simulation) const;

    // Performs the simulation, computes the mean, min, max error based on the generated data.
    // Error can be different from fitness.
    boost::tuple<double,double,double> single_post_evaluation(PaGMOSimulationProportionalDerivative &simulation) const;

    // Number of evaluations an indiviual will be tested in a generation
    unsigned int m_n_evaluations;

    // Time one simulation will be run
    double m_simulation_time;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int) {
        ar & boost::serialization::base_object<base_stochastic>(*this);
        ar & m_n_evaluations;
        ar & m_simulation_time;
    }
};

}} //namespaces

BOOST_CLASS_EXPORT_KEY(pagmo::problem::hovering_problem_proportional_derivative)

#endif // HOVERINGPROBLEMPROPORTIONALDERIVATIVE_H
