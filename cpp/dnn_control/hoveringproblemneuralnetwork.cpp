#include "hoveringproblemneuralnetwork.h"
#include "configuration.h"
#include "constants.h"

#include <limits>

namespace pagmo { namespace problem {

hovering_problem_neural_network::hovering_problem_neural_network(const unsigned int &seed, const unsigned int &n_evaluations, const double &simulation_time, const unsigned int &n_hidden_neurons, const std::set<SensorSimulator::SensorType> &sensor_types, const bool &enable_sensor_noise, const FitnessFunctionType &fitness_function, const PostEvaluationFunctionType &post_evaluation_function, const double &transient_response_time, const double &divergence_set_value)
    : base_stochastic(PaGMOSimulationNeuralNetwork(0, n_hidden_neurons, sensor_types).ChromosomeSize(), seed),
      m_n_evaluations(n_evaluations), m_n_hidden_neurons(n_hidden_neurons), m_simulation_time(simulation_time), m_sensor_types(sensor_types), m_enable_sensor_noise(enable_sensor_noise),
      m_fitness_function(fitness_function), m_post_evaluation_function(post_evaluation_function),
      m_transient_response_time(transient_response_time), m_divergence_set_value(divergence_set_value) {

    set_lb(-1.0);
    set_ub(1.0);
}

hovering_problem_neural_network::hovering_problem_neural_network(const hovering_problem_neural_network &other)
    : base_stochastic(other) {
    m_n_evaluations = other.m_n_evaluations;
    m_simulation_time = other.m_simulation_time;
    m_n_hidden_neurons = other.m_n_hidden_neurons;
    m_sensor_types = other.m_sensor_types;
    m_enable_sensor_noise = other.m_enable_sensor_noise;
    m_fitness_function = other.m_fitness_function;
    m_post_evaluation_function = other.m_post_evaluation_function;
    m_transient_response_time = other.m_transient_response_time;
    m_divergence_set_value = other.m_divergence_set_value;
}

std::string hovering_problem_neural_network::get_name() const {
    return "Asteroid hovering - Neurocontroller Evolution";
}

base_ptr hovering_problem_neural_network::clone() const {
    return base_ptr(new hovering_problem_neural_network(*this));
}

fitness_vector hovering_problem_neural_network::objfun_seeded(const unsigned int &seed, const decision_vector &x) const {
    fitness_vector f(1);

    PaGMOSimulationNeuralNetwork simulation(seed, m_n_hidden_neurons, x, m_sensor_types, m_enable_sensor_noise);
    if (m_simulation_time > 0) {
        simulation.SetSimulationTime(m_simulation_time);
    }

    f[0] = single_fitness(simulation);

    return f;
}

void hovering_problem_neural_network::objfun_impl(fitness_vector &f, const decision_vector &x) const {
    f[0] = 0.0;

    // Make sure the pseudorandom sequence will always be the same
    m_urng.seed(m_seed);

    unsigned int n_evaluations = 0;
    for (unsigned int count = 0; count < m_n_evaluations; count++) {

        // Creates the initial conditions at random, based on the current seed
        const unsigned int current_seed = m_urng();

        // Neural Network simulation
        PaGMOSimulationNeuralNetwork simulation(current_seed, m_n_hidden_neurons, x, m_sensor_types, m_enable_sensor_noise);
        if (m_simulation_time > 0.0) {
            simulation.SetSimulationTime(m_simulation_time);
        }
        f[0] += single_fitness(simulation);
        n_evaluations++;
        if (f[0] > 1e15) {
            break;
        }
    }
    f[0] /= n_evaluations;
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

    switch(m_fitness_function) {

    case FitnessFunctionType::FitnessCompareStartEndPosition:
    {
        // Method 1 : Compare start and ending position and velocity
        const Vector3D &position_end = evaluated_positions.back();
        const Vector3D &velocity_end = evaluated_velocities.back();
        fitness += VectorNorm(VectorSub(target_position, position_end)) + VectorNorm(velocity_end);
    }
        break;

    case FitnessFunctionType::FitnessAveragePositionOffset:
    {
        // Method 2 : Compare mean distance to target point. Transient response aware
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
                fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i)));
                considered_samples++;
            }
        }
        fitness /= considered_samples;
    }
        break;

    case FitnessFunctionType::FitnessAveragePositionOffsetAndVelocity:
    {
        // Method 3 : Compare mean distance to target point, also consider velocity. Transient response aware.
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
                fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i))) + VectorNorm(evaluated_velocities.at(i));
                considered_samples++;
            }
        }
        fitness /= considered_samples;
    }
        break;

    case FitnessFunctionType::FitnessAveragePositionOffsetAndFuel:
    {
        // Method 4 : Compare mean distance to target point, also consider fuel consumption. Transient response aware.
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
                fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i)));
                considered_samples++;
            }
        }
        fitness /= considered_samples;
        fitness += 200.0 * (simulation.SpacecraftMaximumMass() / evaluated_masses.back() - 1.0);
    }
        break;

    case FitnessFunctionType::FitnessAveragePositionOffsetAndVelocityAndFuel:
    {
        // Method 5 : Compare mean distance to target point, also consider velocity, also consider fuel consumption. Transient response aware.
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
                fitness += VectorNorm(VectorSub(target_position, evaluated_positions.at(i))) + VectorNorm(evaluated_velocities.at(i));
                considered_samples++;
            }
        }
        fitness /= considered_samples;
        fitness += 200.0 * (simulation.SpacecraftMaximumMass() / evaluated_masses.back() - 1.0);
    }
        break;

    case FitnessFunctionType::FitnessAverageVelocity:
    {
        // Method 6 : Mean velocity. Transient response aware.
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
                fitness += VectorNorm(evaluated_velocities.at(i));
                considered_samples++;
            }
        }
        fitness /= considered_samples;
    }
        break;

    case FitnessFunctionType::FitnessAverageOpticFlowAndConstantDivergence:
    {
        // Method 7 : Mean optical flow, constant divergence. Transient response aware.
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
                const Vector3D &height = evaluated_heights.at(i);
                const Vector3D &velocity = evaluated_velocities.at(i);

                const double coef_norm_height = 1.0 / VectorNorm(height);
                const Vector3D normalized_height = VectorMul(coef_norm_height, height);

                const double magn_velocity_parallel = VectorDotProduct(velocity, normalized_height);
                const double divergence = magn_velocity_parallel * coef_norm_height;

                const Vector3D velocity_parallel = VectorMul(magn_velocity_parallel, normalized_height);
                const Vector3D velocity_perpendicular = VectorSub(velocity, velocity_parallel);

                const Vector3D &optic_flow = VectorMul(coef_norm_height, velocity_perpendicular);

                double error_divergence = divergence + m_divergence_set_value;
                error_divergence = (error_divergence < 0.0 ? -error_divergence : error_divergence);

                const double error_optical_flow = VectorNorm(optic_flow);

                fitness += error_optical_flow + error_divergence;

                considered_samples++;
            }
        }
        fitness /= considered_samples;
    }
        break;

    default:
        throw FitnessFunctionTypeNotImplemented();

    }

    return fitness;
}

boost::tuple<double, double, double, double, double> hovering_problem_neural_network::single_post_evaluation(PaGMOSimulationNeuralNetwork &simulation) const {
    double mean_error = 0.0;
    double min_error = std::numeric_limits<double>::max();
    double max_error = -std::numeric_limits<double>::max();
    double predicted_fuel = 0.0;
    double used_fuel = 0.0;

    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > result = simulation.EvaluateAdaptive();
    const std::vector<double> &evaluated_times = boost::get<0>(result);
    const std::vector<double> &evaluated_masses = boost::get<1>(result);
    const std::vector<Vector3D> &evaluated_positions = boost::get<2>(result);
    const std::vector<Vector3D> &evaluated_heights = boost::get<3>(result);
    const std::vector<Vector3D> &evaluated_velocities = boost::get<4>(result);
    const std::vector<std::vector<double> > &evaluated_accelerations = boost::get<6>(result);

    const unsigned int num_samples = evaluated_times.size();

    // The fuel consumption
    const double dt = 1.0 / simulation.ControlFrequency();
    const double coef = 1.0 / (simulation.SpacecraftSpecificImpulse() * kEarthAcceleration);
    unsigned int samples = 0;
    int index = -1;
    for (unsigned int i = 0; i < num_samples; ++i) {
        if (evaluated_times.at(i) >= m_transient_response_time) {
            if (index == -1) {
                index = i;
            }
            const Vector3D &acc = {evaluated_accelerations.at(i).at(0), evaluated_accelerations.at(i).at(1), evaluated_accelerations.at(i).at(2)};
            predicted_fuel += dt * VectorNorm(acc) * evaluated_masses.at(i) * coef;
            samples++;
        }
    }
    used_fuel = evaluated_masses.at(index) - evaluated_masses.at(num_samples - 1);

    // The target position
    const Vector3D target_position = simulation.TargetPosition();

    // punish unfinished simulations (crash / out of fuel)
    // punish unfinished simulations (crash / out of fuel)
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

    switch (m_post_evaluation_function) {

        case PostEvaluationFunctionType::PostEvalAveragePositionOffset:
    {
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
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
    }
        break;

    case PostEvaluationFunctionType::PostEvalAverageVelocity:
    {
        // Compare mean velocity. Transient response aware.
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= m_transient_response_time) {
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
    }
        break;

    default:
        throw PostEvaluationFunctionTypeNotImplemented();
    }

    return boost::make_tuple(mean_error, min_error, max_error, predicted_fuel, used_fuel);
}

boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> >, std::vector<std::pair<double, double> > > hovering_problem_neural_network::post_evaluate(const decision_vector &x, const unsigned int &start_seed, const std::vector<unsigned int> &random_seeds) const {
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

    const double simulation_time = 3600;
    std::vector<double> mean_errors(num_tests, 0.0);
    std::vector<std::pair<double, double> > min_max_errors(num_tests);
    std::vector<std::pair<double, double> > fuel_consumption(num_tests);

    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int current_seed = used_random_seeds.at(i);

        PaGMOSimulationNeuralNetwork simulation(current_seed, m_n_hidden_neurons, x, m_sensor_types, m_enable_sensor_noise, {SensorSimulator::SensorType::ExternalAcceleration});
        simulation.SetSimulationTime(simulation_time);

        const boost::tuple<double, double, double, double, double> result = single_post_evaluation(simulation);

        mean_errors.at(i) = boost::get<0>(result);
        min_max_errors.at(i).first = boost::get<1>(result);
        min_max_errors.at(i).second = boost::get<2>(result);
        fuel_consumption.at(i).first = boost::get<3>(result);
        fuel_consumption.at(i).second = boost::get<4>(result);
    }

    return boost::make_tuple(used_random_seeds, mean_errors, min_max_errors, fuel_consumption);
}

}}
