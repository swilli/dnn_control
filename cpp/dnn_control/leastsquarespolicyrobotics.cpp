#include "leastsquarespolicyrobotics.h"
#include "constants.h"
#include "vector.h"
#include "lspisimulator.h"
#include "filewriter.h"
#include "configuration.h"

#include <cfloat>
#include <eigen3/Eigen/Sparse>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <iomanip>
#include <fstream>
#include <limits>

#include <thread>

enum MDPState {
    RelativeLocalisation,
    Velocity,
    EMD,
    EMDAndAccelerometer
};


static const MDPState kMDPState = MDPState::RelativeLocalisation;
static const unsigned int kSpacecraftStateDimension = 6;

typedef boost::array<double, kSpacecraftStateDimension> LSPIState;


static unsigned int kSpacecraftPhiSize = 0;

// (x, a, r, x_prime)
typedef boost::tuple<LSPIState, unsigned int, double, LSPIState> Sample;

static std::vector<Vector3D> kSpacecraftActions;

static void Init() {
    const std::vector<double> t = {-21.0, -15.0, -8.0, -3.0, -1.0, -0.5, -0.1, 0.0, 0.1, 0.5, 1.0, 3.0, 8.0, 15.0, 21.0};
    for (unsigned int i = 0; i < t.size(); ++i) {
        for (unsigned int j = 0; j < t.size(); ++j) {
            for (unsigned int k = 0; k < t.size(); ++k) {
                const Vector3D thrust = {t[i], t[j], t[k]};
                kSpacecraftActions.push_back(thrust);
            }
        }
    }
    kSpacecraftPhiSize = 64;
}

static Eigen::VectorXd Phi(const LSPIState &state, const unsigned int &action_index) {
    Eigen::VectorXd result(kSpacecraftPhiSize);

    unsigned int base = 0;

    const Vector3D &action = kSpacecraftActions[action_index];

    result(base++) = 1.0;
    for (unsigned int i = 0; i < state.size(); ++i) {
        for (unsigned int k = 0; k < 3; ++k) {
            result(base++) = state[i] * action[k];
        }
    }
    for (unsigned int i = 0; i < state.size(); ++i) {
        for (unsigned int j = 0; j < state.size(); ++j) {
            result(base++) = state[i] * state[j];
        }
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            result(base++) = action[i] * action[j];
        }
    }
    return result;
}

static unsigned int Pi(SampleFactory &sample_factory, const LSPIState &state, const Eigen::VectorXd &weights) {
    std::vector<unsigned int> best_a;
    double best_q = -std::numeric_limits<double>::max();
    for (unsigned int a = 0; a < kSpacecraftActions.size(); ++a) {
        Eigen::VectorXd val_phi = Phi(state, a);
        Eigen::VectorXd val_phi_t = val_phi.transpose();

        const double q = val_phi_t.dot(weights);
        if (q > best_q) {
            best_q = q;
            best_a.clear();
            best_a.push_back(a);
        } else if (q == best_q) {
            best_a.push_back(a);
        }
    }
    return best_a.at(sample_factory.SampleRandomNatural() % best_a.size());
}

static Eigen::VectorXd LSTDQ(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const Eigen::VectorXd &weights) {
    Eigen::MatrixXd matrix_A(kSpacecraftPhiSize, kSpacecraftPhiSize);
    Eigen::VectorXd vector_b(kSpacecraftPhiSize);

    for (unsigned int i = 0; i < samples.size(); ++i) {
        const Sample &sample = samples.at(i);
        const LSPIState &s = boost::get<0>(sample);
        const LSPIState &s_prime = boost::get<3>(sample);
        const unsigned int &a = boost::get<1>(sample);
        const double &r = boost::get<2>(sample);

        const Eigen::VectorXd phi_sa = Phi(s, a);
        const unsigned int a_prime = Pi(sample_factory, s_prime, weights);
        const Eigen::VectorXd phi_sa_prime = Phi(s_prime, a_prime);

        matrix_A = matrix_A + phi_sa * (phi_sa - gamma * phi_sa_prime).transpose();
        vector_b = vector_b + r * phi_sa;
    }

    return matrix_A.colPivHouseholderQr().solve(vector_b);
}

static void PLSTDQThreadFun(const unsigned int &seed, const std::vector<Sample> &samples, const unsigned int &start_index, const unsigned int &end_index, const double &gamma, const Eigen::VectorXd &weights, Eigen::MatrixXd *matrix_A, Eigen::VectorXd *vector_b) {
    SampleFactory sample_factory(seed);

    matrix_A->setZero();
    vector_b->setZero();

    for (unsigned int i = start_index; i < end_index; ++i) {
        const Sample &sample = samples.at(i);
        const LSPIState &s = boost::get<0>(sample);
        const LSPIState &s_prime = boost::get<3>(sample);
        const unsigned int &a = boost::get<1>(sample);
        const double &r = boost::get<2>(sample);

        const Eigen::VectorXd phi_sa = Phi(s, a);
        const unsigned int a_prime = Pi(sample_factory, s_prime, weights);
        const Eigen::VectorXd phi_sa_prime = Phi(s_prime, a_prime);

        *matrix_A = *matrix_A + phi_sa * (phi_sa - gamma * phi_sa_prime).transpose();
        *vector_b = *vector_b + r * phi_sa;
    }
}

static Eigen::VectorXd PLSTDQ(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const Eigen::VectorXd &weights) {
    const unsigned int seed1 = sample_factory.SampleRandomNatural();
    const unsigned int seed2 = sample_factory.SampleRandomNatural();

    Eigen::MatrixXd matrix1(kSpacecraftPhiSize, kSpacecraftPhiSize);
    Eigen::MatrixXd matrix2(kSpacecraftPhiSize, kSpacecraftPhiSize);

    Eigen::VectorXd vector1(kSpacecraftPhiSize);
    Eigen::VectorXd vector2(kSpacecraftPhiSize);

    Eigen::MatrixXd matrix_A(kSpacecraftPhiSize, kSpacecraftPhiSize);
    Eigen::VectorXd vector_b(kSpacecraftPhiSize);

    const unsigned half = samples.size()/2;

    std::thread thread1(PLSTDQThreadFun, seed1, samples, 0, half, gamma, weights, &matrix1, &vector1);
    std::thread thread2(PLSTDQThreadFun, seed2, samples, half, samples.size(), gamma, weights, &matrix2, &vector2);

    thread1.join();
    thread2.join();

    matrix_A = matrix1 + matrix2;
    vector_b = vector1 + vector2;

    return matrix_A.colPivHouseholderQr().solve(vector_b);
}

static Eigen::VectorXd LSPI(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const double &epsilon, const Eigen::VectorXd &initial_weights) {
    Eigen::VectorXd w_prime(initial_weights);
    Eigen::VectorXd w;

    double val_norm = -1.0;
    unsigned int iteration = 0;
    do {
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::cout << std::endl << asctime(timeinfo) << "iteration " << iteration++ << ". Norm : " << val_norm << std::endl;

        w = w_prime;
        w_prime = PLSTDQ(sample_factory, samples, gamma, w);
        val_norm = (w - w_prime).norm();
    } while (val_norm > epsilon);

    return w;
}

static LSPIState SystemStateToLSPIState(SampleFactory &sample_factory, const Asteroid &asteroid, const double &time, const Vector3D &perturbations_acceleration, const SystemState &state, const Vector3D &target_position) {
    LSPIState lspi_state;

    const Vector3D &position = {state[0], state[1], state[2]};
    const Vector3D &velocity = {state[3], state[4], state[5]};

    const boost::tuple<Vector3D, double> result = asteroid.NearestPointOnSurfaceToPosition(position);
    const Vector3D &surf_pos = boost::get<0>(result);
    const double norm_height = boost::get<1>(result);
    const Vector3D height = VectorSub(position, surf_pos);

    switch(kMDPState) {
    case RelativeLocalisation:
        for (unsigned int i = 0; i < 3; ++i) {
            lspi_state[i] = target_position[i] - position[i];
            lspi_state[3+i] = -velocity[i];

            lspi_state[i] +=  lspi_state[i] * sample_factory.SampleNormal(0.0, 0.05);
            lspi_state[3+i] += lspi_state[3+i] * sample_factory.SampleNormal(0.0, 0.05);
        }

        break;

    case Velocity:
        for (unsigned int i = 0; i < 3; ++i) {
            lspi_state[i] = -velocity[i];
            lspi_state[i] +=  lspi_state[i] * sample_factory.SampleNormal(0.0, 0.05);
        }

        break;

    case EMD:
    case EMDAndAccelerometer:
        const double coef_norm_height = 1.0 / norm_height;
        const Vector3D normalized_height = VectorMul(coef_norm_height, height);
        const double magn_velocity_parallel = VectorDotProduct(velocity, normalized_height);
        const Vector3D velocity_parallel = VectorMul(magn_velocity_parallel, normalized_height);
        const Vector3D velocity_perpendicular = VectorSub(velocity, velocity_parallel);

        for (unsigned int i = 0; i < 3; ++i) {
            lspi_state[i] = velocity_parallel[i] * coef_norm_height;
            lspi_state[i] +=  lspi_state[i] * sample_factory.SampleNormal(0.0, 0.05);
        }
        for (unsigned int i = 0; i < 3; ++i) {
            lspi_state[3 + i] = velocity_perpendicular[i] * coef_norm_height;
            lspi_state[3 + i] +=  lspi_state[3 + i] * sample_factory.SampleNormal(0.0, 0.05);
        }

        if (kMDPState == EMDAndAccelerometer) {
            const Vector3D gravity_acceleration = asteroid.GravityAccelerationAtPosition(position);

            const boost::tuple<Vector3D, Vector3D> result_angular = asteroid.AngularVelocityAndAccelerationAtTime(time);
            const Vector3D &angular_velocity = boost::get<0>(result_angular);
            const Vector3D &angular_acceleration = boost::get<1>(result_angular);
            const Vector3D euler_acceleration = VectorCrossProduct(angular_acceleration, position);
            const Vector3D centrifugal_acceleration = VectorCrossProduct(angular_velocity, VectorCrossProduct(angular_velocity, position));
            const Vector3D coriolis_acceleration = VectorCrossProduct(VectorMul(2.0, angular_velocity), velocity);

            Vector3D acceleration;
            for (unsigned int i = 0; i < 3; ++i) {
                double value = perturbations_acceleration[i]
                        + gravity_acceleration[i]
                        - coriolis_acceleration[i]
                        - euler_acceleration[i]
                        - centrifugal_acceleration[i];

                acceleration[i] = value + value * sample_factory.SampleNormal(0.0, 0.05);
            }

            const double coef_norm_height = 1.0 / norm_height;
            const Vector3D normalized_height = VectorMul(coef_norm_height, height);
            const double magn_acceleration_parallel = VectorDotProduct(acceleration, normalized_height);

            lspi_state[6] = (magn_acceleration_parallel < 0.0 ? -magn_acceleration_parallel : magn_acceleration_parallel);
        }
        break;

    }

    return lspi_state;
}

SystemState InitializeState(SampleFactory &sample_factory, const Vector3D &target_position, const double &mass, const double &maximum_position_offset, const double &maximum_velocity) {
    SystemState system_state;
    Vector3D position = target_position;
    for (unsigned int i = 0; i < 3; ++i) {
        position[i] += sample_factory.SampleUniformReal(-maximum_position_offset, maximum_position_offset);
    }
    for (unsigned int i = 0; i < 3; ++i) {
        system_state[i] = position[i];
        system_state[i+3] = sample_factory.SampleUniformReal(-maximum_velocity, maximum_velocity);
    }
    system_state[6] = mass;

    return system_state;
}

static std::vector<Sample> PrepareSamples(SampleFactory &sample_factory, const unsigned int &num_samples, const unsigned int &num_steps) {
    std::vector<Sample> samples;

    for (unsigned int i = 0; i < num_samples; ++i) {
#ifdef LSPR_FIXED_SEED
        LSPISimulator simulator(LSPR_FIXED_SEED);
        const Vector3D target_position = simulator.SampleFactoryOfSystem().SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
#else
        LSPISimulator simulator(sample_factory.SampleRandomNatural());
        const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
        const Vector3D &target_position = boost::get<0>(sampled_point);
#endif
        const double dt = 1.0 / simulator.ControlFrequency();

        Asteroid &asteroid = simulator.AsteroidOfSystem();
        SystemState state = InitializeState(sample_factory, target_position, simulator.SpacecraftMaximumMass(), sample_factory.SampleBoolean() * 10.0, sample_factory.SampleBoolean() * 1.0);
        double time = sample_factory.SampleUniformReal(0.0, 12.0 * 60.0 * 60.0);
        Vector3D perturbations_acceleration = simulator.RefreshPerturbationsAcceleration();
        LSPIState lspi_state = SystemStateToLSPIState(sample_factory, asteroid, time, perturbations_acceleration, state, target_position);

        for (unsigned int j = 0; j < num_steps; ++j) {
            const Vector3D &position = {state[0], state[1], state[2]};
            const Vector3D &velocity = {state[3], state[4], state[5]};

            const unsigned int a = sample_factory.SampleUniformNatural(0, kSpacecraftActions.size() - 1);
            const Vector3D &thrust = kSpacecraftActions[a];
            const boost::tuple<SystemState, Vector3D, double, bool> result = simulator.NextState(state, time, thrust);
            const bool exception = boost::get<3>(result);
            if (exception) {
                std::cout << "sample sequence stopped." << std::endl;
                break;
            }
            const SystemState &next_state = boost::get<0>(result);
            time += dt;

            const Vector3D &next_position = {next_state[0], next_state[1], next_state[2]};
            const Vector3D &next_velocity = {next_state[3], next_state[4], next_state[5]};

            perturbations_acceleration = simulator.RefreshPerturbationsAcceleration();
            const LSPIState next_lspi_state = SystemStateToLSPIState(sample_factory, asteroid, time, perturbations_acceleration, next_state, target_position);

            const double delta_p1 = VectorNorm(VectorSub(target_position, position));
            const double delta_p2 = VectorNorm(VectorSub(target_position, next_position));
            const double delta_v1 = VectorNorm(velocity);
            const double delta_v2 = VectorNorm(next_velocity);

            const double r = delta_p1 - delta_p2 + delta_v1 - delta_v2;

            samples.push_back(boost::make_tuple(lspi_state, a, r, next_lspi_state));

            state = next_state;
            lspi_state = next_lspi_state;
        }
    }
    return samples;
}


static boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluatePolicy(SampleFactory &sample_factory, const Eigen::VectorXd &weights, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<double> evaluated_times(num_steps + 1);
    std::vector<double> evaluated_masses(num_steps + 1);
    std::vector<Vector3D> evaluated_positions(num_steps + 1);
    std::vector<Vector3D> evaluated_heights(num_steps + 1);
    std::vector<Vector3D> evaluated_velocities(num_steps + 1);
    std::vector<Vector3D> evaluated_thrusts(num_steps + 1);
    std::vector<Vector3D> evaluated_accelerations(num_steps + 1);

    Asteroid &asteroid = simulator.AsteroidOfSystem();
    const double dt = 1.0 / simulator.ControlFrequency();

#if LSPR_IC_VELOCITY_TYPE == LSPR_IC_BODY_ZERO_VELOCITY
    SystemState state = InitializeState(sample_factory, target_position, simulator.SpacecraftMaximumMass(),  LSPR_IC_POSITION_OFFSET_ENABLED * 3.0, 0.0);
#elif LSPR_IC_VELOCITY_TYPE == LSPR_IC_BODY_RANDOM_VELOCITY
    SystemState state = InitializeState(sample_factory, target_position, simulator.SpacecraftMaximumMass(), LSPR_IC_POSITION_OFFSET_ENABLED * 3.0, 0.3);
#endif
    double time = 0.0;

    Vector3D thrust = {0.0, 0.0, 0.0};

    unsigned int iteration;
    bool exception_thrown = false;
    double time_observer = 0.0;
    Vector3D perturbations_acceleration = simulator.RefreshPerturbationsAcceleration();
    for (iteration = 0; iteration < num_steps; ++iteration) {
        const Vector3D &position = {state[0], state[1], state[2]};
        const Vector3D &velocity = {state[3], state[4], state[5]};
        const double &mass = state[6];

        const Vector3D surface_point = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
        const Vector3D &height = {position[0] - surface_point[0], position[1] - surface_point[1], position[2] - surface_point[2]};

        const LSPIState lspi_state = SystemStateToLSPIState(sample_factory, asteroid, time, perturbations_acceleration, state, target_position);

        thrust = kSpacecraftActions[Pi(sample_factory, lspi_state, weights)];

        const boost::tuple<SystemState, Vector3D, double, bool> result  = simulator.NextState(state, time, thrust);
        const SystemState next_state = boost::get<0>(result);
        const Vector3D &acceleration = boost::get<1>(result);
        time_observer = boost::get<2>(result);
        exception_thrown = boost::get<3>(result);

        evaluated_times.at(iteration) = time;
        evaluated_masses.at(iteration) = mass;
        evaluated_positions.at(iteration) = position;
        evaluated_heights.at(iteration) = height;
        evaluated_velocities.at(iteration) = velocity;
        evaluated_thrusts.at(iteration) = thrust;
        evaluated_accelerations.at(iteration) = acceleration;

        state = next_state;
        time += dt;
        perturbations_acceleration = simulator.RefreshPerturbationsAcceleration();
        if (exception_thrown) {
            std::cout << "spacecraft crash or out of fuel." << std::endl;
            break;
        }
    }
    if(exception_thrown) {
        const unsigned int new_size = iteration + 2;
        evaluated_times.resize(new_size);
        evaluated_masses.resize(new_size);
        evaluated_positions.resize(new_size);
        evaluated_velocities.resize(new_size);
        evaluated_heights.resize(new_size);
        evaluated_thrusts.resize(new_size);
    }

    const Vector3D &position = {state[0], state[1], state[2]};
    const Vector3D &velocity = {state[3], state[4], state[5]};
    const double &mass = state[6];

    const Vector3D surf_pos = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
    const Vector3D &height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    evaluated_times.back() = time_observer;
    evaluated_masses.back() = mass;
    evaluated_positions.back() = position;
    evaluated_velocities.back() = velocity;
    evaluated_heights.back() = height;
    evaluated_thrusts.back() = thrust;

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_thrusts, evaluated_accelerations);
}

static boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> >, std::vector<std::pair<double, double> > > PostEvaluateLSPIController(const Eigen::VectorXd &controller_weights, const unsigned int &start_seed, const std::vector<unsigned int> &random_seeds=std::vector<unsigned int>()) {
    unsigned int num_tests = random_seeds.size();
    std::vector<unsigned int> used_random_seeds;
    if (num_tests == 0) {
        SampleFactory sample_factory(start_seed);
        num_tests = 10000;
        for (unsigned int i = 0; i < num_tests; ++i) {
            used_random_seeds.push_back(sample_factory.SampleRandomNatural());
        }
    } else {
        used_random_seeds = random_seeds;
    }

    std::vector<double> mean_errors(num_tests, 0.0);
    std::vector<std::pair<double, double> > min_max_errors(num_tests);
    std::vector<std::pair<double, double> > fuel_consumptions(num_tests);

    const double test_time = 3600.0;

    for (unsigned int i = 0; i < num_tests; ++i) {
        std::cout << "test " << (i+1) << ":" << std::endl;
        const unsigned int current_seed = used_random_seeds.at(i);
        LSPISimulator simulator(current_seed);
        SampleFactory &sample_factory = simulator.SampleFactoryOfSystem();
        const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
        const Vector3D &target_position = boost::get<0>(sampled_point);

        const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = EvaluatePolicy(sample_factory, controller_weights, simulator, target_position, test_time * simulator.ControlFrequency());
        const std::vector<double> &evaluated_times = boost::get<0>(result);
        const std::vector<double> &evaluated_masses = boost::get<1>(result);
        const std::vector<Vector3D> &evaluated_positions = boost::get<2>(result);
        const std::vector<Vector3D> &evaluated_accelerations = boost::get<6>(result);

        const unsigned int num_samples = evaluated_times.size();

        double predicted_fuel = 0.0;
        const double dt = 1.0 / simulator.ControlFrequency();
        const double coef = 1.0 / (simulator.SpacecraftSpecificImpulse() * kEarthAcceleration);
        int index = -1;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= LSPR_TRANSIENT_RESPONSE_TIME) {
                if (index == -1) {
                    index = i;
                }
                predicted_fuel += dt * VectorNorm(evaluated_accelerations.at(i)) * evaluated_masses.at(i) * coef;
            }
        }
        double used_fuel = evaluated_masses.at(index) - evaluated_masses.at(num_samples - 1);

        double mean_error = 0.0;
        double min_error = std::numeric_limits<double>::max();
        double max_error = -std::numeric_limits<double>::max();

        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= LSPR_TRANSIENT_RESPONSE_TIME) {
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
        mean_errors.at(i) = mean_error;
        min_max_errors.at(i).first = min_error;
        min_max_errors.at(i).second = max_error;
        fuel_consumptions.at(i).first = predicted_fuel;
        fuel_consumptions.at(i).second = used_fuel;
    }

    return boost::make_tuple(used_random_seeds, mean_errors, min_max_errors, fuel_consumptions);
}

void TestLeastSquaresPolicyController(const unsigned int &random_seed) {
    ConfigurationLSPI();

    Init();

    const unsigned int worst_case_seed = 457110846;

    const double test_time = 2.0 * 24.0 * 60.0 * 60.0;

    LSPISimulator simulator(worst_case_seed);
    SampleFactory &sample_factory = simulator.SampleFactoryOfSystem();
    const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
    const Vector3D &target_position = boost::get<0>(sampled_point);

    Eigen::VectorXd weights(kSpacecraftPhiSize);

    std::ifstream weight_file(PATH_TO_LSPI_WEIGHT_VECTOR_FILE);
    double weight = 0;
    unsigned int i = 0;
    while (weight_file >> weight) {
        weights[i++] = weight;
    }
    weight_file.close();

    std::cout << "Simulating LSPI controller ... ";
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = EvaluatePolicy(sample_factory, weights, simulator, target_position, test_time * simulator.ControlFrequency());
    const std::vector<double> &times = boost::get<0>(result);
    const std::vector<Vector3D> &positions = boost::get<2>(result);
    const std::vector<Vector3D> &heights = boost::get<3>(result);
    const std::vector<Vector3D> &velocities = boost::get<4>(result);
    const std::vector<Vector3D> &thrusts = boost::get<5>(result);
    std::cout << "done." << std::endl;

    std::cout << "Writing visualization file ... ";
    FileWriter writer_visualization(PATH_TO_LSPI_TRAJECTORY_FILE);
    writer_visualization.CreateTrajectoryFile(simulator.ControlFrequency(), simulator.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;

    std::cout << "Writing evaluation file ... ";
    FileWriter writer_evaluation(PATH_TO_LSPI_EVALUATION_FILE);
    writer_evaluation.CreateEvaluationFile(random_seed, target_position, simulator.AsteroidOfSystem(), times, positions, velocities, thrusts);
    std::cout << "done." << std::endl;


    std::cout << "Performing post evaluation ... ";
    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> >, std::vector<std::pair<double, double> > > post_evaluation = PostEvaluateLSPIController(weights, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<0>(post_evaluation);
    const std::vector<double> &mean_errors = boost::get<1>(post_evaluation);
    const std::vector<std::pair<double, double > > &min_max_errors = boost::get<2>(post_evaluation);
    const std::vector<std::pair<double, double > > &fuel_consumptions = boost::get<3>(post_evaluation);

    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_LSPI_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, mean_errors, min_max_errors, fuel_consumptions);
    std::cout << "done." << std::endl;
}

void TrainLeastSquaresPolicyController() {
    ConfigurationLSPI();

    std::cout << "Initializing LSPI controller learning .... ";
    Init();

#if LSPR_WRITE_ACTION_SET_TO_FILE
    FileWriter writer(PATH_TO_LSPI_ACTION_SET_FILE);
    writer.CreateActionSetFile(kSpacecraftActions);
#endif

    const unsigned int num_samples = LSPR_NUM_EPISODES;
    const unsigned int num_steps = LSPR_NUM_STEPS;

    const double gamma = LSPR_GAMMA;
    const double epsilon = LSPR_EPSILON;

    SampleFactory sample_factory(rand());

    const std::vector<Sample> samples = PrepareSamples(sample_factory, num_samples, num_steps);

    std::cout << std::setprecision(10);

    std::cout << "collected " << samples.size() << " samples." << std::endl;

    Eigen::VectorXd weights(kSpacecraftPhiSize);
    weights.setZero();
    weights = LSPI(sample_factory, samples, gamma, epsilon, weights);

    std::cout << "solution:" << std::endl;
    std::cout << weights[0];
    for (unsigned int i = 1; i < weights.rows(); ++i) {
        std::cout << ", " << weights[i];
    }
    std::cout << std::endl;
    std::cout << "creating lspi weights file ... ";
    FileWriter weights_writer(PATH_TO_LSPI_WEIGHT_VECTOR_FILE);
    weights_writer.CreateLSPIWeightsFile(weights);
    std::cout << "done." << std::endl;
}
