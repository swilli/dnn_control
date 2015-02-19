#include "hoveringproblemfullstate.h"
#include "configuration.h"

#include <limits>

namespace pagmo { namespace problem {

hovering_problem_full_state::hovering_problem_full_state(const unsigned int &seed, const unsigned int &n_evaluations, const double &simulation_time)
    : base_stochastic(PaGMOSimulationFullState(0, 0.0).ChromosomeSize(), seed),
      m_n_evaluations(n_evaluations), m_simulation_time(simulation_time) {

    set_lb(-1.0);
    set_ub(1.0);
}

hovering_problem_full_state::hovering_problem_full_state(const hovering_problem_full_state &other)
    : base_stochastic(other) {
    m_n_evaluations = other.m_n_evaluations;
    m_simulation_time = other.m_simulation_time;
}

std::string hovering_problem_full_state::get_name() const {
    return "Asteroid hovering - PID controller Evolution";
}

base_ptr hovering_problem_full_state::clone() const {
    return base_ptr(new hovering_problem_full_state(*this));
}

fitness_vector hovering_problem_full_state::objfun_seeded(const unsigned int &seed, const decision_vector &x) const {
    m_seed = seed;
    fitness_vector f(1);
    objfun_impl(f, x);
    return f;
}

void hovering_problem_full_state::objfun_impl(fitness_vector &f, const decision_vector &x) const {
    f[0] = 0.0;

    // Make sure the pseudorandom sequence will always be the same
    m_urng.seed(m_seed);

    for (unsigned int count = 0; count < m_n_evaluations; count++) {

        // Creates the initial conditions at random, based on the current seed
#ifdef HP_FIXED_SEED
        const unsigned int current_seed = HP_FIXED_SEED;
#else
        const unsigned int current_seed = m_urng();
#endif

        // PD Controller simulation
        PaGMOSimulationFullState simulation(current_seed, m_simulation_time, x);
        f[0] += single_fitness(simulation);
    }
    f[0] /= m_n_evaluations;
}

std::string hovering_problem_full_state::human_readable_extra() const {
    std::ostringstream oss;
    oss << "\tSimulation Time: " << m_simulation_time << '\n';
    oss << "\tSeed: " << m_seed << '\n';
    oss << "\tSample Size: " << m_n_evaluations << '\n';
    return oss.str();
}

double hovering_problem_full_state::single_fitness(PaGMOSimulationFullState &simulation) const {
    double fitness = 0.0;

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.EvaluateAdaptive();
    const std::vector<double> &evaluated_times = boost::get<0>(result);
    const std::vector<double> &evaluated_masses = boost::get<1>(result);
    const std::vector<Vector3D> &evaluated_positions = boost::get<2>(result);
    const std::vector<Vector3D> &evaluated_heights = boost::get<3>(result);
    const std::vector<Vector3D> &evaluated_velocities = boost::get<4>(result);

    const unsigned int num_samples = evaluated_times.size();

    // The target position
    const Vector3D target_position = simulation.TargetPosition();

    // punish unfinished simulations (crash / out of fuel)
#if HP_OBJ_FUN_PUNISH_UNFINISHED_SIMULATIONS_ENABLED
    double time_diff = evaluated_times.back() - m_simulation_time;
    time_diff = (time_diff < 0.0 ? -time_diff : time_diff);
    if (time_diff > 0.1) {
        fitness += 1e30;
    }
#endif

#if HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_1
    // Method 1 : Compare start and ending position and velocity
    const Vector3D &position_end = evaluated_positions.back();
    const Vector3D &velocity_end = evaluated_velocities.back();
    fitness += VectorNorm(VectorSub(target_position, position_end)) + VectorNorm(velocity_end);

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_2
    // Method 2 : Compare mean distance to target point. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i)));
            considered_samples++;
        }
    }
    fitness /= considered_samples;

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_3
    // Method 3 : Compare mean distance to target point, also consider velocity. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i))) + VectorNorm(evaluated_velocities.at(i));
            considered_samples++;
        }
    }
    fitness /= considered_samples;

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_4
    // Method 4 : Compare mean distance to target point, also consider fuel consumption. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i)));
            considered_samples++;
        }
    }
    fitness /= considered_samples;
    fitness += 1.0 / (evaluated_masses.back() - simulation.SpacecraftMinimumMass() + 0.001);

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_5
    // Method 5 : Compare mean distance to target point, also consider velocity, punish later offsets more.
    for (unsigned int i = 0; i < num_samples; ++i) {
        fitness += (i + 1) * (VectorNorm(VectorSub(target_position, evaluated_positions.at(i))) + VectorNorm(evaluated_velocities.at(i)));
    }
    fitness /= num_samples;

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_6
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            fitness += VectorNorm(evaluated_velocities.at(i));
            considered_samples++;
        }
    }
    fitness /= considered_samples;

#endif

    return fitness;
}

boost::tuple<double, double, double> hovering_problem_full_state::single_post_evaluation(PaGMOSimulationFullState &simulation) const {
    double mean_error = 0.0;
    double min_error = std::numeric_limits<double>::max();
    double max_error = -std::numeric_limits<double>::max();

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = simulation.EvaluateAdaptive();
    const std::vector<double> &evaluated_times = boost::get<0>(result);
    const std::vector<Vector3D> &evaluated_positions = boost::get<2>(result);
    const std::vector<Vector3D> &evaluated_heights = boost::get<3>(result);

    const unsigned int num_samples = evaluated_times.size();

    // The target position
    const Vector3D target_position = simulation.TargetPosition();

    // punish unfinished simulations (crash / out of fuel)
#if HP_OBJ_FUN_PUNISH_UNFINISHED_SIMULATIONS_ENABLED
    double time_diff = evaluated_times.back() - m_simulation_time;
    time_diff = (time_diff < 0.0 ? -time_diff : time_diff);
    if (time_diff > 0.1) {
        mean_error += 1e30;
    }
#endif

    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            const double error = VectorNorm(VectorSub(target_position, evaluated_positions.at(i)));
            if (error > max_error) {
                max_error = error;
            } else if(error < min_error) {
                min_error = error;
            }
            mean_error += error;
            considered_samples++;
        }
    }
    mean_error /= considered_samples;

    return boost::make_tuple(mean_error, min_error, max_error);
}

hovering_problem_full_state::~hovering_problem_full_state() {

}

boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> > > hovering_problem_full_state::post_evaluate(const decision_vector &x, const unsigned int &start_seed, const std::vector<unsigned int> &random_seeds) const {
    unsigned int num_tests = random_seeds.size();
    std::vector<unsigned int> used_random_seeds;
    if (num_tests == 0) {
        m_urng.seed(start_seed);
        num_tests = 25000;
        for (unsigned int i = 0; i < num_tests; ++i) {
            used_random_seeds.push_back(m_urng());
        }
    } else {
        used_random_seeds = random_seeds;
    }

    std::vector<double> mean_errors(num_tests, 0.0);
    std::vector<std::pair<double,double> > min_max_errors(num_tests, std::make_pair(0.0, 0.0));

    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int current_seed = used_random_seeds.at(i);

        PaGMOSimulationFullState simulation(current_seed, m_simulation_time, x);

        const boost::tuple<double, double, double> result = single_post_evaluation(simulation);

        mean_errors.at(i) = boost::get<0>(result);
        min_max_errors.at(i).first = boost::get<1>(result);
        min_max_errors.at(i).second = boost::get<2>(result);
    }

    return boost::make_tuple(used_random_seeds, mean_errors, min_max_errors);
}

}}
