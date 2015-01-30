#include "leastsquarespolicyrobotics.h"
#include "vector.h"
#include "lspisimulator.h"
#include "filewriter.h"

#include <cfloat>
#include <eigen3/Eigen/Dense>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <iomanip>

#define LSPI_TEST_OUTPUT_FILE   "../../../results/visualization_lspi.txt"

namespace eigen = Eigen;

static const double kSpacecraftMaximumThrust = 21.0;

static const unsigned int kSpacecraftStateDimension = 6;

static unsigned int kSpacecraftNumActions = 0;
static unsigned int kSpacecraftPolynomialDimensions = 0;
static unsigned int kSpacecraftPhiSize = 0;


// delta r, delta dot r
typedef boost::array<double, kSpacecraftStateDimension> LSPIState;

// (x, a, r, x_prime)
typedef boost::tuple<LSPIState, unsigned int, double, LSPIState> Sample;

static std::map<int, Vector3D> spacecraft_actions;

static void Init() {
    unsigned int index = 0;
    const Vector3D actions_1d = {-kSpacecraftMaximumThrust, 0.0, kSpacecraftMaximumThrust};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                const Vector3D action = {actions_1d[i], actions_1d[j], actions_1d[k]};
                spacecraft_actions[index++] = action;
            }
        }
    }

    /*
    spacecraft_actions[index++] = {0.0, 0.0, 0.0};
    spacecraft_actions[index++] = {-kSpacecraftMaximumThrust, 0.0, 0.0};
    spacecraft_actions[index++] = {kSpacecraftMaximumThrust, 0.0, 0.0};
    spacecraft_actions[index++] = {0.0, -kSpacecraftMaximumThrust, 0.0};
    spacecraft_actions[index++] = {0.0, kSpacecraftMaximumThrust, 0.0};
    spacecraft_actions[index++] = {0.0, 0.0, -kSpacecraftMaximumThrust};
    spacecraft_actions[index++] = {0.0, 0.0, kSpacecraftMaximumThrust};
    */
    kSpacecraftNumActions = index;
    kSpacecraftPolynomialDimensions = (int) (0.5 * kSpacecraftStateDimension * (kSpacecraftStateDimension + 3) + 1);
    kSpacecraftPhiSize = kSpacecraftNumActions * kSpacecraftPolynomialDimensions;
}

static eigen::VectorXd Phi(const LSPIState &state, const unsigned int &action) {
    eigen::VectorXd result = eigen::VectorXd(kSpacecraftPhiSize);
    result.setZero();
    unsigned int base = action * kSpacecraftPolynomialDimensions;

    result[base++] = 1.0;
    for (unsigned int i = 0; i < kSpacecraftStateDimension; ++i) {
        result[base++] = state[i];
        result[base++] = state[i] * state[i];
        for (unsigned int j = i+1; j < kSpacecraftStateDimension; ++j) {
            result[base++] = state[i] * state[j];
        }
    }

    return result;
}

static unsigned int Pi(SampleFactory &sample_factory, const LSPIState &state, const eigen::VectorXd &weights) {
    std::vector<unsigned int> best_a;
    double best_q = -DBL_MAX;
    for (unsigned int a = 0; a < kSpacecraftNumActions; ++a) {
        eigen::VectorXd  val_phi = Phi(state, a);
        eigen::VectorXd  val_phi_t = val_phi.transpose();

        double q = val_phi_t.dot(weights);
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

static eigen::VectorXd LSTDQ(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const eigen::VectorXd &weights) {
    eigen::MatrixXd matrix_A(kSpacecraftPhiSize, kSpacecraftPhiSize);
    matrix_A.setZero();
    eigen::VectorXd vector_b(kSpacecraftPhiSize);
    vector_b.setZero();

    for (unsigned int i = 0; i < samples.size(); ++i) {
        const Sample sample = samples.at(i);
        const LSPIState s = boost::get<0>(sample);
        const LSPIState s_prime = boost::get<3>(sample);
        const unsigned int a = boost::get<1>(sample);
        const double r = boost::get<2>(sample);

        const eigen::VectorXd phi_sa = Phi(s, a);
        const unsigned int a_prime = Pi(sample_factory, s_prime, weights);
        const eigen::VectorXd phi_sa_prime = Phi(s_prime, a_prime);

        matrix_A = matrix_A + phi_sa * (phi_sa - gamma * phi_sa_prime).transpose();
        vector_b = vector_b + r * phi_sa;
    }

    return matrix_A.inverse() * vector_b;
}

static eigen::VectorXd LSPI(SampleFactory &sample_factory, const std::vector<Sample> &samples, const double &gamma, const double &epsilon, const eigen::VectorXd &initial_weights) {
    eigen::VectorXd w_prime(initial_weights);
    eigen::VectorXd w;

    double val_norm = 0.0;
    unsigned int iteration = 0;
    do {
        flush(std::cout);
        w = w_prime;
        w_prime = LSTDQ(sample_factory, samples, gamma, w);
        val_norm = (w - w_prime).norm();
        std::cout << "iteration " << iteration++ << ". Norm : " << val_norm << std::endl;
    } while (val_norm > epsilon);

    return w;
}

static LSPIState Sys2LSPI(const SystemState &state, const Vector3D &target_position, const double &time, const Asteroid &asteroid) {
    LSPIState lspi_state;

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};

    for (unsigned int k = 0; k < 3; ++k) {
        lspi_state[k] = target_position[k] - position[k];
    }

    const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(time));
    const Vector3D target_zero_velocity = VectorSub({0.0, 0.0, 0.0}, VectorCrossProduct(angular_velocity, target_position));

    for (unsigned int k = 0; k < 3; ++k) {
        lspi_state[3+k] = target_zero_velocity[k] - velocity[k];
    }
    return lspi_state;
}

static boost::tuple<SystemState, double> InitializeState(SampleFactory &sample_factory, const Vector3D &target_position, const Asteroid &asteroid) {
    SystemState result;
    Vector3D position;
    for (unsigned int i = 0; i < 3; ++i) {
        position[i] = sample_factory.SampleUniform(target_position[i] - 3.0, target_position[i] + 3.0);
    };
    const double mass = sample_factory.SampleUniform(450.0, 550.0);
    const double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);
    const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(time));
    const Vector3D velocity = VectorCrossProduct(angular_velocity, position);
    for (unsigned int i = 0; i < 3; ++i) {
        result[i] = position[i];
        result[i+3] = -velocity[i] + sample_factory.SampleUniform(-0.3, 0.3);
    }
    result[6] = mass;
    return boost::make_tuple(result, time);
}

std::vector<Sample> PrepareSamples(const unsigned int &seed, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_samples, const unsigned int &num_steps) {
    std::vector<Sample> samples;

    Asteroid &asteroid = simulator.AsteroidOfSystem();

    const double dt = simulator.InteractionInterval();
    SampleFactory sample_factory(seed);
    for (unsigned int i = 0; i < num_samples; ++i) {
        boost::tuple<SystemState, double> result = InitializeState(sample_factory, target_position, asteroid);
        SystemState state = boost::get<0>(result);
        double time = boost::get<1>(result);

        for (unsigned int j = 0; j < num_steps; ++j) {
            const Vector3D position = {state[0], state[1], state[2]};
            const LSPIState lspi_state = Sys2LSPI(state, target_position, time, asteroid);

            const unsigned int a = sample_factory.SampleRandomInteger() % kSpacecraftNumActions;
            const Vector3D &thrust = spacecraft_actions[a];
            const boost::tuple<SystemState, bool> result = simulator.NextState(state, time, thrust);
            const bool exception = boost::get<1>(result);
            if (exception) {
                break;
            }
            SystemState next_state = boost::get<0>(result);

            const Vector3D next_position = {next_state[0], next_state[1], next_state[2]};

            time += dt;

            const LSPIState next_lspi_state = Sys2LSPI(next_state, target_position, time, asteroid);

            double error_state = 0;
            double error_next_state = 0;
            for (unsigned int k = 0; k < 3; ++k) {
                error_state += (position[k] - target_position[k]) * (position[k] - target_position[k]);
                error_next_state += (next_position[k] - target_position[k]) * (next_position[k] - target_position[k]);
            }
            error_state = sqrt(error_state);
            error_next_state = sqrt(error_next_state);

            const double r = error_state - error_next_state;

            samples.push_back(boost::make_tuple(lspi_state, a, r, next_lspi_state));
            state = next_state;
        }
    }
    return samples;
}


static void TestPolicy(const unsigned int &seed, const eigen::VectorXd &weights, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<Vector3D> positions;
    std::vector<Vector3D> heights;

    Asteroid &asteroid = simulator.AsteroidOfSystem();
    const double dt = simulator.InteractionInterval();

    SampleFactory sample_factory(seed);

    const boost::tuple<SystemState, double> result = InitializeState(sample_factory, target_position, asteroid);

    SystemState state = boost::get<0>(result);
    double time = boost::get<1>(result);

    std::cout << "testing policy ... ";
    for (unsigned int i = 0; i < num_steps; ++i) {
        const Vector3D position = {state[0], state[1], state[2]};
        const Vector3D surface_point = boost::get<0>(simulator.AsteroidOfSystem().NearestPointOnSurfaceToPosition(position));
        const Vector3D height = {position[0] - surface_point[0], position[1] - surface_point[1], position[2] - surface_point[2]};

        positions.push_back(position);
        heights.push_back(height);

        const LSPIState lspi_state = Sys2LSPI(state, target_position, time, asteroid);

        const Vector3D thrust = spacecraft_actions[Pi(sample_factory, lspi_state, weights)];
        const boost::tuple<SystemState, bool> result  = simulator.NextState(state, time, thrust);
        const bool exception_thrown = boost::get<1>(result);
        if (exception_thrown) {
            std::cout << "spacecraft crash or out of fuel." << std::endl;
            break;
        }
        state = boost::get<0>(result);
        time += dt;
    }

    std::cout << "done." << std::endl << "writing result to file ... ";
    FileWriter writer;
    writer.CreateVisualizationFile(LSPI_TEST_OUTPUT_FILE, simulator.InteractionInterval(), asteroid, positions, heights);
    std::cout << "done." << std::endl;
}

void TestPolicySolution() {
    Init();
    const unsigned int random_seed = 0;
    const double test_time = 24.0 * 60.0 * 60.0;

    SampleFactory sample_factory(random_seed);
    LSPISimulator simulator(sample_factory.SampleRandomInteger());
    const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    const unsigned int test_seed = sample_factory.SampleRandomInteger();

    TestPolicy(test_seed, weights, simulator, target_position, test_time / simulator.InteractionInterval());
}

void TrainLeastSquaresPolicyController(const unsigned int &random_seed) {
    std::cout << "LSPI seed: " << random_seed << std::endl;

    Init();
    const unsigned int num_samples = 1000;
    const unsigned int num_steps = 300;

    const double gamma = 0.9;
    const double epsilon = 1e-5;

    SampleFactory sample_factory(random_seed);
    LSPISimulator simulator(sample_factory.SampleRandomInteger());
    const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);

    const std::vector<Sample> samples = PrepareSamples(sample_factory.SampleRandomInteger(), simulator, target_position, num_samples, num_steps);

    std::cout << std::setprecision(10);

    std::cout << "collected " << samples.size() << " samples." << std::endl;

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights.setZero();
    weights = LSPI(sample_factory, samples, gamma, epsilon, weights);

    std::cout << "solution:" << std::endl << "<< ";
    for (unsigned int i = 0; i < weights.rows() - 1; ++i) {
        std::cout << weights[i] << ", ";
    }
    std::cout << weights[weights.rows()-1] << ";" << std::endl;
}
