#include "hoveringproblem.h"
#include "pagmosimulationfullstate.h"
#include "pagmosimulationneuralnetwork.h"

namespace pagmo { namespace problem {

hovering_problem::hovering_problem(const unsigned int &seed, const unsigned int &n_evaluations, const double &simulation_time, const unsigned int &n_hidden_neurons)
    : base_stochastic((n_sensor_dimensions + 1) * n_hidden_neurons + (n_hidden_neurons + 1) * n_control_dimensions, seed),
      m_n_evaluations(n_evaluations), m_n_hidden_neurons(n_hidden_neurons), m_simulation_time(simulation_time) {

    set_lb(-1);
    set_ub(1);

}

hovering_problem::hovering_problem(const hovering_problem &other)
    : base_stochastic(other) {
    m_n_evaluations = other.m_n_evaluations;
    m_simulation_time = other.m_simulation_time;
    m_n_hidden_neurons = other.m_n_hidden_neurons;
}

std::string hovering_problem::get_name() const {
    return "Asteroid hovering - Neurocontroller Evolution";
}

base_ptr hovering_problem::clone() const {
    return base_ptr(new hovering_problem(*this));
}

void hovering_problem::objfun_impl(fitness_vector &f, const decision_vector &x) const {
    f[0] = 0.0;
    // Make sure the pseudorandom sequence will always be the same
    m_drng.seed(m_seed);

    // Set the ffnn weights from x, by accounting for symmetries in neurons weights
    //set_nn_weights(x);

    for (unsigned int count = 0; count < m_n_evaluations; count++) {

        // Creates the initial conditions at random, based on the current seed
        const unsigned int current_seed = m_drng();

        double obj_val = 0.0;

        // Neural Network simulation
        PaGMOSimulationNeuralNetwork simulation(current_seed, m_simulation_time, x, m_n_hidden_neurons);

        const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.Evaluate();
        const std::vector<double> &times = boost::get<0>(result);
        const std::vector<double> &masses = boost::get<1>(result);
        const std::vector<Vector3D> &positions = boost::get<2>(result);
        const std::vector<Vector3D> &velocities = boost::get<4>(result);

        const Vector3D &position_begin = positions[0];

        const unsigned int num_samples = times.size();

        // punish unfinished simulations (crash / out of fuel)
        double time_diff = times.back() - m_simulation_time;
        time_diff = (time_diff < 0.0 ? -time_diff : time_diff);
        if (time_diff > 0.1) {
            obj_val += 1e4;
        }

        // Method 1 : Compare start and ending position and velocity
        // const Vector3D &position_end = positions.back();
        // const Vector3D &velocity_end = velocities.back();
        // obj_val += VectorNorm(VectorSub(position_begin, position_end)) + VectorNorm(velocity_end);


        // Method 2 : Compare mean distance to target point
        for (unsigned int i = 1; i < num_samples; ++i) {
            obj_val += VectorNorm(VectorSub(position_begin, positions.at(i)));
        }
        obj_val /= num_samples - 1;

        // Method 3 : Compare mean distance to target point, also consider velocity
        //for (unsigned int i = 1; i < num_samples; ++i) {
        //	obj_val += VectorNorm(VectorSub(position_begin, positions.at(i))) + VectorNorm(velocities.at(i));
        //}
        //obj_val /= num_samples - 1;

        f[0] += obj_val;
    }
    f[0] /= m_n_evaluations;
}

std::string hovering_problem::human_readable_extra() const {
    std::ostringstream oss;
    oss << "\tSimulation Time: " << m_simulation_time << '\n';
    oss << "\tSeed: " << m_seed << '\n';
    oss << "\tSample Size: " << m_n_evaluations << '\n';
    oss << "\tHidden Neurons: " << m_n_hidden_neurons << '\n';
    return oss.str();
}

hovering_problem::~hovering_problem() {

}

}}
