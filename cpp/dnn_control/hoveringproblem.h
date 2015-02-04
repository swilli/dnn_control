#ifndef HOVERINGPROBLEM_H
#define HOVERINGPROBLEM_H

#include "pagmosimulationneuralnetwork.h"

#include <pagmo/src/problem/base_stochastic.h>
#include <boost/serialization/access.hpp>

#define OBJ_FUN_METHOD_1    1   // Compare start and ending position and velocity.
#define OBJ_FUN_METHOD_2    2   // Compare mean distance to target point.
#define OBJ_FUN_METHOD_3    3   // Compare mean distance to target point, also consider velocity.
#define OBJ_FUN_METHOD_4    4   // Compare mean distance to target point, but don't take into consideration some amount of starting positions.
#define OBJ_FUN_METHOD_5    5   // Compare mean distance to target point, but don't take into consideration some amount of starting positions. Additionally, take into consideration total fuel consumption.
#define OBJ_FUN_METHOD_6    6   // Compare mean distance to target point, also consider velocity, but don't take into consideration some amount of starting positions.
#define OBJ_FUN_METHOD_7    7   // Compare mean distance to target point, also consider velocity, punish later offsets more.

#define OBJECTIVE_FUNCTION_METHOD  OBJ_FUN_METHOD_3

#define PROBLEM_FIXED_SEED  1990

namespace pagmo { namespace problem {

class __PAGMO_VISIBLE hovering_problem : public base_stochastic {
public:
    // Make sure it matches the sensor's class "kDimensions" constant
    static const unsigned int n_sensor_dimensions = 3;

    // Make sure it machtes the output of the controller's "GetThrustForSensorData" function
    static const unsigned int n_control_dimensions = 3;

    hovering_problem(const unsigned int &seed=0, const unsigned int &n_evaluations=5, const double &simulation_time=60.0, const unsigned int &n_hidden_neurons=10);

    hovering_problem(const hovering_problem &other);

    ~hovering_problem();

    std::string get_name() const;

    base_ptr clone() const;

protected:
    void objfun_impl(fitness_vector &f, const decision_vector &x) const;

    std::string human_readable_extra() const;

private:
    double single_fitness(PaGMOSimulationNeuralNetwork &simulation) const;

    unsigned int m_n_evaluations;
    unsigned int m_n_hidden_neurons;
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

BOOST_CLASS_EXPORT_KEY(pagmo::problem::hovering_problem)

#endif // HOVERINGPROBLEM_H
