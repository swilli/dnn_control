#include "leastsquarespolicyiteration.h"
#include "vector.h"
#include "utility.h"
#include "simulator.h"
#include "filewriter.h"

#include <cfloat>
#include <eigen3/Eigen/Dense>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <iomanip>

namespace eigen = Eigen;

#define SPACECRAFT_MAX_THRUST                   50.0
#define SPACECRAFT_STATE_DIMENSION              10

static unsigned int kSpacecraftNumActions = 0;
static unsigned int kSpacecraftPolynomialDimensions = 0;
static unsigned int kSpacecraftPhiSize = 0;

typedef boost::array<double, SPACECRAFT_STATE_DIMENSION> LSPIState;
typedef boost::tuple<LSPIState, unsigned int, double, LSPIState> Sample;

static std::map<int, Vector3D> spacecraft_actions;

static void Init() {
    unsigned int index = 0;
    const Vector3D actions_1d = {-SPACECRAFT_MAX_THRUST, 0.0, SPACECRAFT_MAX_THRUST};
    /*for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                const Vector3D action = {actions_1d[i], actions_1d[j], actions_1d[k]};
                spacecraft_actions[index++] = action;
            }
        }
    }
    */
    spacecraft_actions[index++] = {0.0, 0.0, 0.0};
    spacecraft_actions[index++] = {-SPACECRAFT_MAX_THRUST, 0.0, 0.0};
    spacecraft_actions[index++] = {SPACECRAFT_MAX_THRUST, 0.0, 0.0};
    spacecraft_actions[index++] = {0.0, -SPACECRAFT_MAX_THRUST, 0.0};
    spacecraft_actions[index++] = {0.0, SPACECRAFT_MAX_THRUST, 0.0};
    spacecraft_actions[index++] = {0.0, 0.0, -SPACECRAFT_MAX_THRUST};
    spacecraft_actions[index++] = {0.0, 0.0, SPACECRAFT_MAX_THRUST};

    kSpacecraftNumActions = index;
    kSpacecraftPolynomialDimensions = (int) (0.5 * SPACECRAFT_STATE_DIMENSION * (SPACECRAFT_STATE_DIMENSION + 3) + 1);
    kSpacecraftPhiSize = kSpacecraftNumActions * kSpacecraftPolynomialDimensions;
}

static eigen::VectorXd Phi(const LSPIState &state, const unsigned int &action) {
    eigen::VectorXd result = eigen::VectorXd(kSpacecraftPhiSize);
    result.setZero();
    unsigned int base = action * kSpacecraftPolynomialDimensions;

    result[base++] = 1.0;
    for (unsigned int i = 0; i < SPACECRAFT_STATE_DIMENSION; ++i) {
        result[base++] = state[i];
        result[base++] = state[i] * state[i];
        for (unsigned int j = i+1; j < SPACECRAFT_STATE_DIMENSION; ++j) {
            result[base++] = state[i] * state[j];
        }
    }

    return result;
}

static unsigned int Pi(const LSPIState &state, const eigen::VectorXd &weights) {
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
    return best_a.at(rand() % best_a.size());
}

static eigen::VectorXd LSTDQ(const std::vector<Sample> &samples, const double &gamma, const eigen::VectorXd &weights) {
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
        const unsigned int a_prime = Pi(s_prime, weights);
        const eigen::VectorXd phi_sa_prime = Phi(s_prime, a_prime);

        matrix_A = matrix_A + phi_sa * (phi_sa - gamma * phi_sa_prime).transpose();
        vector_b = vector_b + r * phi_sa;
    }

    return matrix_A.inverse() * vector_b;
}

static eigen::VectorXd LSPI(const std::vector<Sample> &samples, const double &gamma, const double &epsilon, const eigen::VectorXd &initial_weights) {
    unsigned int iteration = 0;

    eigen::VectorXd w_prime(initial_weights);
    eigen::VectorXd w;

    double val_norm = 1.0;
    while (val_norm > epsilon) {
        std::cout << "iteration " << iteration++ << ". Norm : " << val_norm << std::endl;
        flush(std::cout);
        w = w_prime;
        w_prime = LSTDQ(samples, gamma, w);
        val_norm = (w - w_prime).norm();
    }

    std::cout << "iteration " << iteration++ << ". Norm : " << val_norm << std::endl;

    return w;
}

static boost::tuple<LSPIState, double> InitializeState(const Vector3D &target_position, const Asteroid &asteroid) {
    LSPIState result;
    Vector3D position;
    Vector3D velocity;
    for (unsigned int i = 0; i < 3; ++i) {
        position[i] = SampleUniform(target_position[i] - 3.0, target_position[i] + 3.0);
        velocity[i] = SampleUniform(-0.3, 0.3);
    };
    const double mass = SampleUniform(450.0, 550.0);
    const double time = SampleUniform(0.0, 24.0 * 60.0 * 60.0);
    const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(time));
    for (unsigned int i = 0; i < 3; ++i) {
        result[i] = position[i];
        result[i+3] = velocity[i];
        result[i+7] = angular_velocity[i];
    }
    result[6] = mass;

    return boost::make_tuple(result, time);
}

static boost::tuple<std::vector<Sample>, Simulator, Vector3D> PrepareSamples(const unsigned int &num_samples, const unsigned int &num_steps) {
    std::vector<Sample> samples;

    const Vector3D semi_axis = {SampleUniform(8000.0, 12000.0), SampleUniform(4000.0, 7500.0), SampleUniform(1000.0, 3500.0)};
    const double density = SampleUniform(1500.0, 3000.0);
    const Vector2D angular_velocity_xz = {SampleSign() * SampleUniform(0.0002, 0.0008), SampleSign() * SampleUniform(0.0002, 0.0008)};
    const double time_bias = SampleUniform(0.0, 6.0 * 60.0 * 60.0);

    const double spacecraft_specific_impulse = 200.0;

    const double control_frequency = 10.0;

    const double perturbation_noise = 1e-7;
    const double control_noise = 0.05;

    Asteroid asteroid(semi_axis, density, angular_velocity_xz, time_bias);
    Simulator simulator(control_frequency, perturbation_noise, control_noise, asteroid, NULL, NULL, NULL);

    simulator.InitSpacecraftSpecificImpulse(spacecraft_specific_impulse);

    const double dt = 1.0 / control_frequency;

    const Vector3D target_position = SamplePointOutSideEllipsoid(asteroid.SemiAxis(), 4.0);

    for (unsigned int i = 0; i < num_samples; ++i) {
        boost::tuple<LSPIState, double> result = InitializeState(target_position, asteroid);
        LSPIState lspi_state = boost::get<0>(result);

        double time = boost::get<1>(result);
        for (unsigned int j = 0; j < num_steps; ++j) {
            const unsigned int a = rand() % kSpacecraftNumActions;
            const Vector3D &thrust = spacecraft_actions[a];
            const Vector3D position = {lspi_state[0], lspi_state[1], lspi_state[2]};

            State state;
            for (unsigned int k = 0; k < state.size(); ++k) {
                state[k] = lspi_state[k];
            }

            const State next_state = simulator.NextState(state, thrust, time);

            LSPIState next_lspi_state;
            for (unsigned int k = 0; k < state.size(); ++k) {
                next_lspi_state[k] = next_state[k];
            }
            time += dt;
            const Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(time));
            for (unsigned int k = 0; k < 3; ++k) {
                next_lspi_state[7+k] = angular_velocity[k];
            }

            const Vector3D next_position = {next_lspi_state[0], next_lspi_state[1], next_lspi_state[2]};

            double error_state = 0;
            double error_next_state = 0;
            for (unsigned int k = 0; k < 3; ++k) {
                error_state += (position[k] - target_position[k]) * (position[k] - target_position[k]);
                error_next_state += (next_position[k] - target_position[k]) * (next_position[k] - target_position[k]);
            }
            error_state = sqrt(error_state);
            error_next_state = sqrt(error_next_state);

            double r = error_state - error_next_state;
            if (r < 0.0) {
                r = -1.0;
            }

            samples.push_back(boost::make_tuple(lspi_state, a, r, next_lspi_state));
            lspi_state = next_lspi_state;
        }
    }
    return boost::make_tuple(samples, simulator, target_position);
}

static void TestPolicy(const eigen::VectorXd &weights, Simulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<Vector3D> positions;
    std::vector<Vector3D> heights;

    Asteroid &asteroid = simulator.AsteroidOfSystem();

    const boost::tuple<LSPIState,double> result = InitializeState(target_position, asteroid);


    LSPIState lspi_state = boost::get<0>(result);
    State state;
    for (unsigned int i = 0; i < state.size(); ++i) {
        state[i] = lspi_state[i];
    }
    double time = boost::get<1>(result);
    const double dt = simulator.ControlInterval();
    std::cout << "testing policy ... ";
    for (unsigned int i = 0; i < num_steps; ++i) {
        const Vector3D position = {state[0],
                                   state[1],
                                   state[2]};
        const Vector3D surface_point = boost::get<0>(simulator.AsteroidOfSystem().NearestPointOnSurfaceToPosition(position));
        const Vector3D height = {position[0] - surface_point[0],
                                 position[1] - surface_point[1],
                                 position[2] - surface_point[2]};

        positions.push_back(position);
        heights.push_back(height);
        LSPIState lspi_state;
        for (unsigned int k = 0; k < state.size(); ++k) {
            lspi_state[k] = state[k];
        }
        Vector3D angular_velocity = boost::get<0>(asteroid.AngularVelocityAndAccelerationAtTime(time));
        for (unsigned int k = 0; k < 3; ++k) {
            lspi_state[7+k] = angular_velocity[k];
        }
        const Vector3D thrust = spacecraft_actions[Pi(lspi_state, weights)];
        state = simulator.NextState(state, thrust, time);
        time += dt;
    }

    std::cout << "done." << std::endl << "writing result to file ... ";
    FileWriter writer;
    writer.CreateVisualizationFile("../../../results/states.txt", simulator.ControlFrequency(), asteroid, positions, heights);
    std::cout << "done." << std::endl;
}

void LeastSquaresPolicyIteration() {
    Init();
    const unsigned int num_samples = 100;
    const unsigned int num_steps = 300;
    const unsigned int test_time = 24.0 * 60.0 * 60.0;

    const double gamma = 0.9;
    const double epsilon = 1e-20;

    const boost::tuple<std::vector<Sample>, Simulator, Vector3D> result = PrepareSamples(num_samples, num_steps);
    const std::vector<Sample> samples = boost::get<0>(result);
    Simulator simulator = boost::get<1>(result);
    const Vector3D target_position = boost::get<2>(result);

    std::cout << std::setprecision(10);

    std::cout << "collected " << samples.size() << " samples." << std::endl;

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights.setZero();

    weights = LSPI(samples, gamma, epsilon, weights);

    std::cout << weights << std::endl;

    //TestPolicy(weights, simulator, target_position, test_time * simulator.ControlFrequency());
}
