#include "hoveringproblemneuralnetwork.h"
#include "configuration.h"

#include <limits>

namespace pagmo { namespace problem {

hovering_problem_neural_network::hovering_problem_neural_network(const unsigned int &seed, const unsigned int &n_evaluations, const double &simulation_time, const unsigned int &n_hidden_neurons)
    : base_stochastic(PaGMOSimulationNeuralNetwork(0, n_hidden_neurons).ChromosomeSize(), seed),
      m_n_evaluations(n_evaluations), m_n_hidden_neurons(n_hidden_neurons), m_simulation_time(simulation_time) {

    set_lb(-1.0);
    set_ub(1.0);
}

hovering_problem_neural_network::hovering_problem_neural_network(const hovering_problem_neural_network &other)
    : base_stochastic(other) {
    m_n_evaluations = other.m_n_evaluations;
    m_simulation_time = other.m_simulation_time;
    m_n_hidden_neurons = other.m_n_hidden_neurons;
}

std::string hovering_problem_neural_network::get_name() const {
    return "Asteroid hovering - Neurocontroller Evolution";
}

base_ptr hovering_problem_neural_network::clone() const {
    return base_ptr(new hovering_problem_neural_network(*this));
}

fitness_vector hovering_problem_neural_network::objfun_seeded(const unsigned int &seed, const decision_vector &x) const {
    PaGMOSimulationNeuralNetwork simulation(seed, m_n_hidden_neurons, x);
    if (m_simulation_time > 0) {
        simulation.SetSimulationTime(m_simulation_time);
    }
    fitness_vector f(1);
    f[0] = single_fitness(simulation);
    return f;
}

void hovering_problem_neural_network::objfun_impl(fitness_vector &f, const decision_vector &x) const {
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

        // Neural Network simulation
        PaGMOSimulationNeuralNetwork simulation(current_seed, m_n_hidden_neurons, x);
        if (m_simulation_time > 0.0) {
            simulation.SetSimulationTime(m_simulation_time);
        }
        f[0] += single_fitness(simulation);
    }
    f[0] /= m_n_evaluations;
}

std::string hovering_problem_neural_network::human_readable_extra() const {
    std::ostringstream oss;
    oss << "\tSimulation Time: " << m_simulation_time << '\n';
    oss << "\tSeed: " << m_seed << '\n';
    oss << "\tSample Size: " << m_n_evaluations << '\n';
    oss << "\tHidden Neurons: " << m_n_hidden_neurons << '\n';
    return oss.str();
}

double hovering_problem_neural_network::single_fitness(PaGMOSimulationNeuralNetwork &simulation) const {
    double fitness = 0.0;

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > result = simulation.EvaluateAdaptive();
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
    double time_diff = evaluated_times.back() - simulation.SimulationTime();
    time_diff = (time_diff < 0.0 ? -time_diff : time_diff);
    if (time_diff > 0.1) {
        const double norm_height = VectorNorm(evaluated_heights.back());
        if (norm_height < 2.0) {
            const double norm_velocity = VectorNorm(evaluated_velocities.back());
            if (norm_velocity > 0.1) {
                fitness += 1e30;
            } else {
                double error_mass = evaluated_masses.back() - simulation.SpacecraftMinimumMass();
                error_mass = (error_mass < 0.0 ? -error_mass : error_mass);
                if (error_mass < 0.1) {
                    fitness += 1e15;
                }
            }
        } else {
            fitness += 1e30;
        }
    }
#endif

#if HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_1
    // Method 1 : Compare start and ending position and velocity
    const Vector3D &position_end = evaluated_positions.back();
    const Vector3D &velocity_end = evaluated_velocities.back();
    fitness += VectorNorm(VectorSub(target_position, position_end)) + VectorNorm(velocity_end);

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_2
    // Method 2 : Compare mean distance to target point. Transient response aware
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
    fitness += 200.0 * (simulation.SpacecraftMaximumMass() / evaluated_masses.back() - 1.0);

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_5
    // Method 5 : Compare mean distance to target point, also consider velocity, also consider fuel consumption. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i))) + VectorNorm(evaluated_velocities.at(i));
            considered_samples++;
        }
    }
    fitness /= considered_samples;
    fitness += 200.0 * (simulation.SpacecraftMaximumMass() / evaluated_masses.back() - 1.0);

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_6
    // Method 6 : Mean velocity. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            fitness += VectorNorm(evaluated_velocities.at(i));
            considered_samples++;
        }
    }
    fitness /= considered_samples;

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_7
    // Method 7 : Mean optical flow, constant divergence. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            const Vector3D &height = evaluated_heights.at(i);
            const Vector3D &velocity = evaluated_velocities.at(i);

            const double coef_norm_height = 1.0 / VectorNorm(height);
            const Vector3D &normalized_height = {height[0] * coef_norm_height, height[1] * coef_norm_height, height[2] * coef_norm_height};

            const double magn_velocity_parallel = VectorDotProduct(velocity, normalized_height);
            const double divergence = magn_velocity_parallel * coef_norm_height;

            const Vector3D &velocity_vertical = {magn_velocity_parallel * normalized_height[0], magn_velocity_parallel * normalized_height[1], magn_velocity_parallel * normalized_height[2]};
            const Vector3D velocity_horizontal = VectorSub(velocity, velocity_vertical);

            const Vector3D &optical_flow = {velocity_horizontal[0] * coef_norm_height, velocity_horizontal[1] * coef_norm_height, velocity_horizontal[2] * coef_norm_height};

            double error_divergence = divergence + HP_OBJ_FUN_COEF_DIVERGENCE;
            error_divergence = HP_OBJ_FUN_ERROR_DIVERGENCE_WEIGHT * (error_divergence < 0.0 ? -error_divergence : error_divergence);

            const double error_optical_flow = VectorNorm(optical_flow);

            fitness += error_optical_flow + error_divergence;

            considered_samples++;
        }
    }
    fitness /= considered_samples;

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_8
    // Method 8 : Mean offset to optimal landing path.
    Vector3D direction = evaluated_heights.at(0);
    Vector3D landing_point = VectorSub(evaluated_positions.at(0), direction);
    const double dt = 1.0 / simulation.ControlFrequency();
    double t = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        const double magn = exp(-HP_OBJ_FUN_COEF_DIVERGENCE * t);
        const Vector3D target_path_position = VectorAdd(landing_point, VectorMul(magn, direction));
        const Vector3D &actual_position = evaluated_positions.at(i);
        const double error = VectorNorm(VectorSub(target_path_position, actual_position));
        fitness += error;
        t += dt;
    }
    fitness /= num_samples;

#elif HP_OBJECTIVE_FUNCTION_METHOD == HP_OBJ_FUN_METHOD_9
    // Method 9 : Mean distance to target point, target point set to position after the deep controller starts to work.
    Vector3D first_counting_position;
    const double init_phase = 10.0;
    bool initialized = false;
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= init_phase) {
            const Vector3D &actual_position = evaluated_positions.at(i);
            if (!initialized) {
                initialized = true;
                first_counting_position = actual_position;
            }
            const double error = VectorNorm(VectorSub(first_counting_position, actual_position));
            fitness += error;
            considered_samples++;
        }
    }
    fitness /= considered_samples;

#endif

    return fitness;
}

boost::tuple<double, double, double> hovering_problem_neural_network::single_post_evaluation(PaGMOSimulationNeuralNetwork &simulation) const {
    double mean_error = 0.0;
    double min_error = std::numeric_limits<double>::max();
    double max_error = -std::numeric_limits<double>::max();

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > result = simulation.EvaluateAdaptive();
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
    double time_diff = evaluated_times.back() - simulation.SimulationTime();
    time_diff = (time_diff < 0.0 ? -time_diff : time_diff);
    if (time_diff > 0.1) {
        const double norm_height = VectorNorm(evaluated_heights.back());
        if (norm_height < 2.0) {
            const double norm_velocity = VectorNorm(evaluated_velocities.back());
            if (norm_velocity > 0.1) {
                mean_error += 1e30;
            } else {
                double error_mass = evaluated_masses.back() - simulation.SpacecraftMinimumMass();
                error_mass = (error_mass < 0.0 ? -error_mass : error_mass);
                if (error_mass < 0.1) {
                    mean_error += 1e15;
                }
            }
        } else {
            mean_error += 1e30;
        }
    }
#endif

#if HP_POST_EVALUATION_METHOD == HP_POST_EVAL_METHOD_1
    // Compare mean distance to target point. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            const double error = VectorNorm(VectorSub(target_position, evaluated_positions.at(i)));
            if (error > max_error) {
                max_error = error;
                if (min_error == std::numeric_limits<double>::max()) {
                    min_error = max_error;
                }
            } else if(error < min_error) {
                min_error = error;
                if (max_error == -std::numeric_limits<double>::max()){
                    max_error = min_error;
                }
            }
            mean_error += error;
            considered_samples++;
        }
    }
    mean_error /= considered_samples;

#elif HP_POST_EVALUATION_METHOD == HP_POST_EVAL_METHOD_2
    // Compare mean velocity. Transient response aware.
    unsigned int considered_samples = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= HP_OBJ_FUN_TRANSIENT_RESPONSE_TIME) {
            const double error = VectorNorm(evaluated_velocities.at(i));
            if (error > max_error) {
                max_error = error;
                if (min_error == std::numeric_limits<double>::max()) {
                    min_error = max_error;
                }
            } else if(error < min_error) {
                min_error = error;
                if (max_error == -std::numeric_limits<double>::max()){
                    max_error = min_error;
                }
            }
            mean_error += error;
            considered_samples++;
        }
    }
    mean_error /= considered_samples;

#elif HP_POST_EVALUATION_METHOD == HP_POST_EVAL_METHOD_3
    // Compare mean distance to target path.
    Vector3D direction = evaluated_heights.at(0);
    Vector3D landing_point = VectorSub(evaluated_positions.at(0), direction);
    const double dt = 1.0 / simulation.ControlFrequency();
    double t = 0;
    for (unsigned int i = 0; i < num_samples; ++i) {
        const double magn = exp(-HP_OBJ_FUN_COEF_DIVERGENCE * t);
        const Vector3D target_path_position = VectorAdd(landing_point, VectorMul(magn, direction));
        const Vector3D &actual_position = evaluated_positions.at(i);
        const double error = VectorNorm(VectorSub(target_path_position, actual_position));

        if (error > max_error) {
            max_error = error;
            if (min_error == std::numeric_limits<double>::max()) {
                min_error = max_error;
            }
        } else if(error < min_error) {
            min_error = error;
            if (max_error == -std::numeric_limits<double>::max()){
                max_error = min_error;
            }
        }
        mean_error += error;
        t += dt;
    }
    mean_error /= num_samples;
#endif

    return boost::make_tuple(mean_error, min_error, max_error);
}

boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> > > hovering_problem_neural_network::post_evaluate(const decision_vector &x, const unsigned int &start_seed, const std::vector<unsigned int> &random_seeds) const {
    unsigned int num_tests = random_seeds.size();
    std::vector<unsigned int> used_random_seeds;
    if (num_tests == 0) {
        m_urng.seed(start_seed);
        num_tests = 10000;
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

        PaGMOSimulationNeuralNetwork simulation(current_seed, m_n_hidden_neurons, x);
        if (m_simulation_time > 0.0) {
            simulation.SetSimulationTime(m_simulation_time);
        }

        const boost::tuple<double, double, double> result = single_post_evaluation(simulation);

        mean_errors.at(i) = boost::get<0>(result);
        min_max_errors.at(i).first = boost::get<1>(result);
        min_max_errors.at(i).second = boost::get<2>(result);
    }

    return boost::make_tuple(used_random_seeds, mean_errors, min_max_errors);
}

}}
