#ifndef HOVERINGPROBLEMFULLSTATE_H
#define HOVERINGPROBLEMFULLSTATE_H

#include "pagmosimulationfullstate.h"

#include <pagmo/src/problem/base_stochastic.h>
#include <boost/serialization/access.hpp>

namespace pagmo { namespace problem {

class __PAGMO_VISIBLE hovering_problem_full_state : public base_stochastic {
public:
    hovering_problem_full_state(const unsigned int &seed=0, const unsigned int &n_evaluations=4, const double &simulation_time=3600.0);

    hovering_problem_full_state(const hovering_problem_full_state &other);

    ~hovering_problem_full_state();

    boost::tuple<std::vector<double>, std::vector<unsigned int> > post_evaluate(const decision_vector &x, const unsigned int &start_seed=0, const std::vector<unsigned int> &random_seeds=std::vector<unsigned int>()) const;

    std::string get_name() const;

    base_ptr clone() const;

    fitness_vector objfun_seeded(const unsigned int &seed, const decision_vector &x) const;

protected:
    void objfun_impl(fitness_vector &f, const decision_vector &x) const;

    std::string human_readable_extra() const;

private:
    double single_fitness(PaGMOSimulationFullState &simulation) const;
    double single_post_evaluation(PaGMOSimulationFullState &simulation) const;

    unsigned int m_n_evaluations;
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

BOOST_CLASS_EXPORT_KEY(pagmo::problem::hovering_problem_full_state)

#endif // HOVERINGPROBLEMFULLSTATE_H
