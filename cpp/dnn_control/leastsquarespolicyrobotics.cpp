#include "leastsquarespolicyrobotics.h"
#include "constants.h"
#include "vector.h"
#include "lspisimulator.h"
#include "filewriter.h"
#include "configuration.h"

#include <cfloat>
#include <eigen3/Eigen/Dense>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <iomanip>
#include <fstream>
#include <limits>

static const unsigned int kSpacecraftStateDimension = 6;

static unsigned int kSpacecraftNumActions = 0;
static unsigned int kSpacecraftPolynomialDimensions = 0;
static unsigned int kSpacecraftPhiSize = 0;


// delta r, delta dot r
typedef boost::array<double, kSpacecraftStateDimension> LSPIState;

// (x, a, r, x_prime)
typedef boost::tuple<LSPIState, unsigned int, double, LSPIState> Sample;

static std::vector<Vector3D> kSpacecraftActions;

static void Init() {
    const double res_u = 1.0 / LSPR_DIRECTION_RESOLUTION;
    const double res_v = 1.0 / (LSPR_DIRECTION_RESOLUTION - 1);

    const std::vector<double> thrust_levels = {0.0, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 5.0, 10.0, 15.0, 21.0};
    for (unsigned int k = 0; k < thrust_levels.size(); ++k) {
        const double &t = thrust_levels.at(k);
        if (t == 0.0) {
            kSpacecraftActions.push_back({0.0, 0.0, 0.0});
            continue;
        }
        for (unsigned int j = 0; j < LSPR_DIRECTION_RESOLUTION; ++j) {
            if (j == 0) {
                kSpacecraftActions.push_back({0.0, 0.0, -t});
                continue;
            } else if (j == LSPR_DIRECTION_RESOLUTION - 1) {
                kSpacecraftActions.push_back({0.0, 0.0, t});
                continue;
            }
            const double v = j * res_v;
            const double phi = acos(2.0 * v - 1.0);
            for (unsigned int i = 0; i < LSPR_DIRECTION_RESOLUTION; ++i) {
                const double u = i * res_u;
                const double theta = 2.0 * kPi * u;
                const Vector3D action = {t * sin(phi) * cos(theta), t * sin(phi) * sin(theta), t * cos(phi)};
                kSpacecraftActions.push_back(action);
            }
        }
    }
    kSpacecraftNumActions = kSpacecraftActions.size();
    kSpacecraftPolynomialDimensions = 1 + 2 * kSpacecraftStateDimension;
    kSpacecraftPhiSize = kSpacecraftNumActions * kSpacecraftPolynomialDimensions;
}

static Eigen::VectorXd Phi(const LSPIState &state, const unsigned int &action) {
    Eigen::VectorXd result = Eigen::VectorXd(kSpacecraftPhiSize);
    result.setZero();
    unsigned int base = action * kSpacecraftPolynomialDimensions;

    result[base++] = 1.0;
    for (unsigned int i = 0; i < kSpacecraftStateDimension; ++i) {
        result[base++] = state[i];
        result[base++] = state[i] * state[i];
    }

    return result;
}

static unsigned int Pi(SampleFactory &sample_factory, const LSPIState &state, const Eigen::VectorXd &weights) {
    std::vector<unsigned int> best_a;
    double best_q = -std::numeric_limits<double>::max();
    for (unsigned int a = 0; a < kSpacecraftNumActions; ++a) {
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
    return best_a.at(sample_factory.SampleRandomInteger() % best_a.size());
}

static Eigen::VectorXd LSTDQ(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const Eigen::VectorXd &weights) {
    Eigen::MatrixXd matrix_A(kSpacecraftPhiSize, kSpacecraftPhiSize);
    matrix_A.setZero();
    Eigen::VectorXd vector_b(kSpacecraftPhiSize);
    vector_b.setZero();

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

    return matrix_A.inverse() * vector_b;
}

static Eigen::VectorXd LSPI(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const double &epsilon, const Eigen::VectorXd &initial_weights) {
    Eigen::VectorXd w_prime(initial_weights);
    Eigen::VectorXd w;

    double val_norm = 0.0;
    unsigned int iteration = 0;
    do {
        w = w_prime;
        w_prime = LSTDQ(sample_factory, samples, gamma, w);
        val_norm = (w - w_prime).norm();

        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::cout << std::endl << asctime(timeinfo) << "iteration " << ++iteration << ". Norm : " << val_norm << std::endl;
    } while (val_norm > epsilon);

    return w;
}

static LSPIState SystemStateToLSPIState(const SystemState &state, const Vector3D &target_position) {
    LSPIState lspi_state;

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};

    for (unsigned int k = 0; k < 3; ++k) {
        lspi_state[k] = target_position[k] - position[k];
        lspi_state[3+k] = -velocity[k];
    }

    return lspi_state;
}

SystemState InitializeState(SampleFactory &sample_factory, const Vector3D &target_position, const double &maximum_position_offset) {
    SystemState system_state;
    Vector3D position = target_position;
    for (unsigned int i = 0; i < 3; ++i) {
        position[i] += sample_factory.SampleUniform(-maximum_position_offset, maximum_position_offset);
    }
    for (unsigned int i = 0; i < 3; ++i) {
        system_state[i] = position[i];
#if LSPR_IC_VELOCITY_TYPE == LSPR_IC_BODY_ZERO_VELOCITY
        system_state[i+3] = 0.0;
#else
        system_state[i+3] = sample_factory.SampleUniform(-0.3, 0.3);
#endif
    }
    system_state[6] = sample_factory.SampleUniform(450.0, 550.0);

    return system_state;
}

static std::vector<Sample> PrepareSamples(SampleFactory &sample_factory, const unsigned int &num_samples, const unsigned int &num_steps) {
    std::vector<Sample> samples;

    for (unsigned int i = 0; i < num_samples; ++i) {
#ifdef LSPR_FIXED_SEED
        LSPISimulator simulator(LSPR_FIXED_SEED);
        const Vector3D target_position = simulator.SampleFactoryOfSystem().SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
#else
        LSPISimulator simulator(sample_factory.SampleRandomInteger());
        const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
        const Vector3D &target_position = boost::get<0>(sampled_point);
#endif
        const double dt = 1.0 / simulator.ControlFrequency();

        SystemState state = InitializeState(sample_factory, target_position, sample_factory.SampleBoolean() * 6.0);
        double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);

        for (unsigned int j = 0; j < num_steps; ++j) {
            const Vector3D &position = {state[0], state[1], state[2]};

            const LSPIState lspi_state = SystemStateToLSPIState(state, target_position);

            const unsigned int a = sample_factory.SampleRandomInteger() % kSpacecraftNumActions;
            const Vector3D &thrust = kSpacecraftActions[a];
            const boost::tuple<SystemState, double, bool> result = simulator.NextState(state, time, thrust);
            const bool exception = boost::get<2>(result);
            if (exception) {
                break;
            }
            SystemState next_state = boost::get<0>(result);

            const Vector3D &next_position = {next_state[0], next_state[1], next_state[2]};

            const LSPIState next_lspi_state = SystemStateToLSPIState(next_state, target_position);

            const double delta_p1 = VectorNorm(VectorSub(target_position, position));
            const double delta_p2 = VectorNorm(VectorSub(target_position, next_position));

            const double r = -delta_p2;

            samples.push_back(boost::make_tuple(lspi_state, a, r, next_lspi_state));

            time += dt;
            state = next_state;
        }
    }
    return samples;
}


static boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluatePolicy(SampleFactory &sample_factory, const Eigen::VectorXd &weights, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<double> evaluated_times(num_steps + 1);
    std::vector<double> evaluated_masses(num_steps + 1);
    std::vector<Vector3D> evaluated_positions(num_steps + 1);
    std::vector<Vector3D> evaluated_heights(num_steps + 1);
    std::vector<Vector3D> evaluated_velocities(num_steps + 1);
    std::vector<Vector3D> evaluated_thrusts(num_steps + 1);

    Asteroid &asteroid = simulator.AsteroidOfSystem();
    const double dt = 1.0 / simulator.ControlFrequency();

    SystemState state = InitializeState(sample_factory, target_position, LSPR_IC_POSITION_OFFSET_ENABLED * 3.0);
    state[6] = simulator.SpacecraftMaximumMass();
    double time = 0.0;

    Vector3D thrust = {0.0, 0.0, 0.0};

    unsigned int iteration;
    bool exception_thrown = false;
    double time_observer = 0.0;
    for (iteration = 0; iteration < num_steps; ++iteration) {
        const Vector3D &position = {state[0], state[1], state[2]};
        const Vector3D &velocity = {state[3], state[4], state[5]};
        const double &mass = state[6];

        const Vector3D surface_point = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
        const Vector3D &height = {position[0] - surface_point[0], position[1] - surface_point[1], position[2] - surface_point[2]};


        const LSPIState lspi_state = SystemStateToLSPIState(state, target_position);

        thrust = kSpacecraftActions[Pi(sample_factory, lspi_state, weights)];

        evaluated_times.at(iteration) = time;
        evaluated_masses.at(iteration) = mass;
        evaluated_positions.at(iteration) = position;
        evaluated_heights.at(iteration) = height;
        evaluated_velocities.at(iteration) = velocity;
        evaluated_thrusts.at(iteration) = thrust;

        const boost::tuple<SystemState, double, bool> result  = simulator.NextState(state, time, thrust);
        const SystemState next_state = boost::get<0>(result);
        time_observer = boost::get<1>(result);
        exception_thrown = boost::get<2>(result);
        state = next_state;
        time += dt;
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

    return boost::make_tuple(evaluated_times, evaluated_masses, evaluated_positions, evaluated_heights, evaluated_velocities, evaluated_thrusts);
}

static boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double> > > PostEvaluateLSPIController(const Eigen::VectorXd &controller_weights, const unsigned int &start_seed, const std::vector<unsigned int> &random_seeds=std::vector<unsigned int>()) {
    unsigned int num_tests = random_seeds.size();
    std::vector<unsigned int> used_random_seeds;
    if (num_tests == 0) {
        SampleFactory sample_factory(start_seed);
        num_tests = 10000;
        for (unsigned int i = 0; i < num_tests; ++i) {
            used_random_seeds.push_back(sample_factory.SampleRandomInteger());
        }
    } else {
        used_random_seeds = random_seeds;
    }

    std::vector<double> mean_errors(num_tests, 0.0);
    std::vector<std::pair<double, double> > min_max_errors(num_tests);

    const double test_time = 1.0 * 60.0 * 60.0;

    for (unsigned int i = 0; i < num_tests; ++i) {
        const unsigned int current_seed = used_random_seeds.at(i);
        LSPISimulator simulator(current_seed);
        SampleFactory &sample_factory = simulator.SampleFactoryOfSystem();
        const boost::tuple<Vector3D, double, double, double> sampled_point = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
        const Vector3D &target_position = boost::get<0>(sampled_point);

        const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = EvaluatePolicy(sample_factory, controller_weights, simulator, target_position, test_time * simulator.ControlFrequency());
        const std::vector<double> &evaluated_times = boost::get<0>(result);
        const std::vector<Vector3D> &evaluated_positions = boost::get<2>(result);

        const unsigned int num_samples = evaluated_times.size();
        double mean_error = 0.0;
        double min_error = std::numeric_limits<double>::max();
        double max_error = -std::numeric_limits<double>::max();
        unsigned int considered_samples = 0;
        for (unsigned int i = 0; i < num_samples; ++i) {
            if (evaluated_times.at(i) >= LSPR_TRANSIENT_RESPONSE_TIME) {
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
        mean_errors.at(i) = mean_error;
        min_max_errors.at(i).first = min_error;
        min_max_errors.at(i).second = max_error;
    }

    return boost::make_tuple(used_random_seeds, mean_errors, min_max_errors);
}

void TestLeastSquaresPolicyController(const unsigned int &random_seed) {
    ConfigurationLSPI();

    Init();

    const double test_time = 24.0 * 60.0 * 60.0;

    LSPISimulator simulator(random_seed);
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
    const boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > result = EvaluatePolicy(sample_factory, weights, simulator, target_position, test_time * simulator.ControlFrequency());
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
    const boost::tuple<std::vector<unsigned int>, std::vector<double>, std::vector<std::pair<double, double > > > post_evaluation = PostEvaluateLSPIController(weights, random_seed);
    const std::vector<unsigned int> &random_seeds = boost::get<0>(post_evaluation);
    const std::vector<double> &mean_errors = boost::get<1>(post_evaluation);
    const std::vector<std::pair<double, double > > min_max_errors = boost::get<2>(post_evaluation);


    std::cout << "done." << std::endl << "Writing post evaluation file ... ";
    FileWriter writer_post_evaluation(PATH_TO_LSPI_POST_EVALUATION_FILE);
    writer_post_evaluation.CreatePostEvaluationFile(random_seeds, mean_errors, min_max_errors);
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
