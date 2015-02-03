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

        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::cout << std::endl << asctime(timeinfo) << "iteration " << iteration++ << ". Norm : " << val_norm << std::endl;
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
        const Vector3D surface_point = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
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

void TestLeastSquaresPolicyController() {
    Init();
    const unsigned int random_seed = 0;
    const double test_time = 24.0 * 60.0 * 60.0;

    SampleFactory sample_factory(random_seed);
    LSPISimulator simulator(sample_factory.SampleRandomInteger());
    const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights << -140.2008952, -0.2201731214, 7.51365861e-06, -1.45818718e-05, -5.87135368e-06, -0.03597859708, 0.0224278296, 0.01256033236, -0.1237868682, -1.450846915e-05, -3.890676324e-06, -0.01428752027, 0.0107165536, 0.005544998041, -0.02371910865, -3.183836178e-06, -0.002750571038, 0.003523406566, 0.0007791157323, 40.39303309, 6.764918984, 1.416945887, 2.992642533, 30.65395136, -1.86476425, -1.182177289, 19.72571085, 0.4723504367, -141.1337739, -0.2166449612, -5.495659151e-07, -1.83955809e-05, -7.798083117e-06, -0.03334169758, 0.02262430315, 0.01275693829, -0.1230544664, -1.497569197e-05, -4.343944214e-06, -0.01363773056, 0.01077024855, 0.005625824377, -0.02325937989, -3.293799884e-06, -0.002436065626, 0.003556808406, 0.000794810184, 39.83174577, 6.570494694, 1.366311472, 2.984408626, 30.60851032, -1.886570533, -1.196041417, 19.71988107, 0.4690283569, -141.9688734, -0.2160473909, -4.225003942e-08, -1.828241006e-05, -7.746933409e-06, -0.033053381, 0.02241486557, 0.01225596644, -0.1228263885, -1.494557978e-05, -4.338922331e-06, -0.01354222047, 0.01072458857, 0.005473274806, -0.02311150778, -3.298438522e-06, -0.002382710853, 0.003533629697, 0.0007390932682, 39.51037981, 6.523105176, 1.395042288, 3.093995653, 30.67112647, -1.883258373, -1.144481685, 19.87622377, 0.4796667346, -141.5733465, -0.2176945884, 5.063124229e-06, -1.471847263e-05, -6.540660472e-06, -0.03492408866, 0.02231458741, 0.01168160622, -0.1234418337, -1.434969548e-05, -3.930082882e-06, -0.01417709179, 0.01065508243, 0.005296845762, -0.02341211373, -3.226526176e-06, -0.002604235199, 0.003523592003, 0.000672849001, 39.97766962, 6.69099969, 1.412957309, 3.15555826, 30.72162679, -1.881094327, -1.146407851, 19.99915578, 0.4877184944, -142.331355, -0.2182999651, 9.880377688e-08, -1.80248168e-05, -7.536781703e-06, -0.0335047163, 0.02272425054, 0.0123824737, -0.1237671726, -1.49254475e-05, -4.276878439e-06, -0.01370820536, 0.01079804796, 0.005505082572, -0.02350130853, -3.284761406e-06, -0.002467074481, 0.003565915314, 0.0007482484511, 39.73829053, 6.567248626, 1.348794535, 3.04159259, 30.74584902, -1.8884844, -1.183430671, 19.84249721, 0.4745373456, -143.0498734, -0.2190072189, 4.029500736e-07, -1.873320605e-05, -7.63171306e-06, -0.03331608441, 0.02279127662, 0.01191893466, -0.1238909796, -1.514759979e-05, -4.388188691e-06, -0.01353745162, 0.01083072922, 0.00538164901, -0.02350845592, -3.300150207e-06, -0.002419628518, 0.003575653003, 0.0006932658773, 39.62966472, 6.539841159, 1.342995765, 3.134279615, 30.80249646, -1.872173626, -1.135410336, 19.93127617, 0.4715076056, -142.706769, -0.2189677149, 3.815876059e-06, -1.556062324e-05, -6.826397371e-06, -0.0346576347, 0.02238296885, 0.01181590593, -0.123872696, -1.444563213e-05, -4.037402465e-06, -0.01411691312, 0.01064213758, 0.005341649999, -0.02359967343, -3.23758986e-06, -0.002571764283, 0.003527754477, 0.000698525873, 39.91553996, 6.662300733, 1.408530696, 3.137527283, 30.80640715, -1.855669981, -1.168425982, 20.0377645, 0.4843146349, -143.4867731, -0.2180604347, -1.764858922e-06, -1.995311088e-05, -8.121529233e-06, -0.03240695472, 0.02296613947, 0.01227457498, -0.1234099958, -1.526310805e-05, -4.53086744e-06, -0.01323002687, 0.01088428242, 0.005463945275, -0.02343968503, -3.323854942e-06, -0.00230755123, 0.003596423991, 0.0007553956564, 39.40978875, 6.431860934, 1.295015921, 3.059008844, 30.77407451, -1.900740786, -1.153119219, 19.90949948, 0.4734761952, -144.199569, -0.2208342433, 2.819583097e-06, -1.68067734e-05, -7.044416322e-06, -0.03433718999, 0.02239874397, 0.01223192418, -0.1245202266, -1.478562232e-05, -4.168026981e-06, -0.01386490636, 0.01071900136, 0.005466796587, -0.02377063696, -3.267628038e-06, -0.002543022143, 0.003532241645, 0.0007467772578, 39.67265945, 6.616832274, 1.400435253, 3.064225806, 30.97705097, -1.880918743, -1.151471224, 19.90810832, 0.483838265, -141.9502594, -0.2145085935, -1.851352798e-06, -2.016613616e-05, -8.250281805e-06, -0.03240466948, 0.02291918876, 0.0123040631, -0.1218171487, -1.529430315e-05, -4.596065959e-06, -0.01324782392, 0.01087816813, 0.005479138299, -0.02303873992, -3.336666562e-06, -0.002292326338, 0.003590540818, 0.0007544853662, 39.13820351, 6.441542397, 1.335313375, 3.065490001, 30.49912206, -1.881583058, -1.180979348, 19.75714226, 0.4708854319, -142.6928906, -0.2161922563, 2.989889037e-06, -1.678448605e-05, -6.983747485e-06, -0.034149712, 0.02230788233, 0.01211185744, -0.122623479, -1.477183143e-05, -4.167852294e-06, -0.01382973366, 0.01065568619, 0.005439458886, -0.02325134875, -3.247183276e-06, -0.002523907307, 0.003520463316, 0.0007320927736, 39.32115618, 6.599510437, 1.416986212, 3.068027102, 30.55573503, -1.866058446, -1.155180602, 19.75058403, 0.5042830224, -143.346792, -0.2174376501, 3.515627223e-06, -1.623609078e-05, -6.91533138e-06, -0.03463041284, 0.02249777844, 0.01159775232, -0.1230599374, -1.466168204e-05, -4.119745981e-06, -0.01397985348, 0.01073425818, 0.005271199915, -0.02333993803, -3.252084062e-06, -0.002565438035, 0.003548934962, 0.0006655740718, 39.32079647, 6.651442924, 1.405487012, 3.200742655, 30.654715, -1.895235676, -1.15625728, 19.86728268, 0.4976232998, -143.0819833, -0.2157344858, -1.723859219e-06, -1.985966559e-05, -8.127117604e-06, -0.03284306654, 0.02317105785, 0.01251924987, -0.1224210721, -1.52229669e-05, -4.528711281e-06, -0.01338018081, 0.01091209044, 0.005530940943, -0.02324475119, -3.322073771e-06, -0.002360621169, 0.003620252957, 0.0007662866267, 39.28860747, 6.51661697, 1.253363224, 3.011561828, 30.45265321, -1.886745968, -1.17962334, 19.67598342, 0.455854428, -143.8510945, -0.2169886636, -2.202835096e-07, -1.819977733e-05, -7.675570381e-06, -0.033861594, 0.02305115797, 0.01241728276, -0.1230141155, -1.488960371e-05, -4.309395028e-06, -0.01381152469, 0.01084426283, 0.00552243027, -0.0233707911, -3.284583453e-06, -0.002498926886, 0.003599571967, 0.0007695511619, 39.33479345, 6.637357059, 1.280438977, 2.998440419, 30.45868793, -1.87783018, -1.186963512, 19.67934037, 0.4865563597, -144.7902137, -0.215372583, 2.680453393e-08, -1.848465961e-05, -7.64815411e-06, -0.03303280539, 0.0226636474, 0.01280911125, -0.1226195372, -1.503295236e-05, -4.350321807e-06, -0.01348872197, 0.01081019409, 0.005616669151, -0.02312895673, -3.285535879e-06, -0.002389326591, 0.003551199118, 0.0008183016901, 38.84548178, 6.5069966, 1.349865516, 2.952121953, 30.69707404, -1.883617352, -1.188765625, 19.55107272, 0.4886300099, -144.3062406, -0.2151447721, -4.134939649e-06, -2.087984501e-05, -8.759273709e-06, -0.03208811491, 0.02307270128, 0.01221539522, -0.1225133395, -1.535350532e-05, -4.673717083e-06, -0.01320566365, 0.01090065001, 0.005471887357, -0.02320233477, -3.353074833e-06, -0.002261720149, 0.00361256159, 0.0007374389933, 39.12036981, 6.46158149, 1.285041543, 3.086643925, 30.67968043, -1.886178255, -1.170571558, 19.90070717, 0.4674108676, -145.0939116, -0.2166289243, -2.855686396e-06, -1.997790165e-05, -8.426314222e-06, -0.03230075982, 0.02305809994, 0.01261986503, -0.1231967403, -1.526826864e-05, -4.565436611e-06, -0.01325574341, 0.01089219813, 0.005566588939, -0.02333739411, -3.352252734e-06, -0.002285954664, 0.003610588518, 0.0007907704584, 38.98153417, 6.452654448, 1.283640019, 2.982602289, 30.70989365, -1.9031209, -1.197880384, 19.66956355, 0.470709059, -145.6517614, -0.2200613891, 3.806074258e-06, -1.615857602e-05, -6.818308807e-06, -0.03490281884, 0.02249748481, 0.01222789116, -0.1241499315, -1.464257452e-05, -4.094534494e-06, -0.01407235188, 0.01074517604, 0.005419067625, -0.0237070091, -3.243179814e-06, -0.002599902565, 0.003545937425, 0.00074501251, 39.32248173, 6.681452192, 1.405931085, 3.052596139, 30.9355272, -1.914135871, -1.161043217, 19.65065568, 0.4868492923, -143.4701882, -0.2135014585, 1.00393363e-06, -1.787226944e-05, -7.429524576e-06, -0.03370522379, 0.02290578805, 0.01216451038, -0.1216631678, -1.492224865e-05, -4.280980993e-06, -0.01369479321, 0.0108328302, 0.005438339867, -0.02301884199, -3.270105381e-06, -0.002471480954, 0.003586064642, 0.0007185960983, 38.95574554, 6.600115695, 1.286835476, 3.079755758, 30.25070071, -1.891966039, -1.175109475, 19.6066282, 0.4747317745, -144.1519329, -0.2139656503, -2.939555944e-06, -2.039445378e-05, -8.477446062e-06, -0.03259487196, 0.02328591849, 0.01225411173, -0.1216989539, -1.531751011e-05, -4.617443111e-06, -0.01329476857, 0.01094982714, 0.005451134331, -0.02301848154, -3.345434587e-06, -0.002321174891, 0.00364259483, 0.0007422634632, 38.72771157, 6.503864464, 1.252172431, 3.06572654, 30.306859, -1.921201002, -1.150002726, 19.58232364, 0.4869608274, -144.849495, -0.2157053125, 3.376440208e-06, -1.655481902e-05, -7.024186189e-06, -0.03436717676, 0.02287977074, 0.01255425603, -0.1225207727, -1.477690009e-05, -4.173627346e-06, -0.01389084701, 0.01090395083, 0.005598501356, -0.02317149103, -3.263046716e-06, -0.002521844613, 0.003589354295, 0.0007954016203, 38.81507419, 6.636000119, 1.341392479, 3.024720652, 30.58408958, -1.913457007, -1.23760056, 19.55473033, 0.4622866289, -144.6382967, -0.2149699763, 1.856079304e-06, -1.679598982e-05, -7.354391211e-06, -0.03399376576, 0.02210543548, 0.01206088002, -0.12234095, -1.468287141e-05, -4.193895171e-06, -0.01386387034, 0.01059917058, 0.005400437132, -0.02321188929, -3.28873665e-06, -0.002484033519, 0.003504114551, 0.0007167461637, 38.87455237, 6.616790816, 1.442745564, 3.112654036, 30.63850742, -1.875679852, -1.1567993, 19.70240076, 0.4651368022, -145.3113397, -0.2156532801, -2.798283614e-06, -2.042509998e-05, -8.572002203e-06, -0.03276357047, 0.02324875234, 0.01192345481, -0.1224490015, -1.535723175e-05, -4.645906502e-06, -0.01337174001, 0.01096383192, 0.005391350277, -0.02321953209, -3.368472946e-06, -0.002320499005, 0.003635970209, 0.0006993473729, 38.68149557, 6.517365471, 1.278383251, 3.121519081, 30.53078208, -1.899893084, -1.147177356, 19.7463217, 0.4624758025, -146.172949, -0.2147462465, -4.211882066e-06, -2.100944631e-05, -8.749962471e-06, -0.03196534798, 0.02291864352, 0.01234803941, -0.122348333, -1.54253281e-05, -4.683698205e-06, -0.01316996799, 0.0108814054, 0.005496695865, -0.02311098992, -3.372387224e-06, -0.002248313448, 0.003589722704, 0.0007642259728, 38.37947541, 6.441804391, 1.308574086, 3.054524387, 30.59511336, -1.884260271, -1.164119192, 19.62062852, 0.4737788858, -145.7431111, -0.21724474, 5.268399018e-08, -1.867544876e-05, -7.732001847e-06, -0.033781919, 0.02295648651, 0.01266584993, -0.1231009626, -1.511065959e-05, -4.413646632e-06, -0.01368492907, 0.01088041054, 0.005596228159, -0.02352809859, -3.309000931e-06, -0.002467921714, 0.003595914311, 0.000790695674, 38.91565177, 6.599309313, 1.317999053, 2.987248569, 30.6842893, -1.892704538, -1.206654309, 19.60300189, 0.4633731657, -146.4311406, -0.2170066025, 4.414910787e-06, -1.595776075e-05, -6.768751409e-06, -0.03475167394, 0.02264413144, 0.01194187686, -0.1229392324, -1.463200721e-05, -4.11209691e-06, -0.01396574551, 0.01075677245, 0.005356902724, -0.02344856907, -3.243340797e-06, -0.002571957772, 0.003567986769, 0.0006978041308, 38.88963295, 6.67889452, 1.323641989, 3.090271489, 30.60401624, -1.908903556, -1.177946315, 19.64340377, 0.4787750898, -147.2061997, -0.2168800837, 2.306732707e-06, -1.684335388e-05, -7.18906954e-06, -0.03418053012, 0.02250191664, 0.01234285406, -0.1230270606, -1.473578136e-05, -4.185106643e-06, -0.01381183903, 0.01075882822, 0.005468735222, -0.02336296296, -3.269975503e-06, -0.002506510047, 0.003543772023, 0.0007712314081, 38.69015569, 6.639121951, 1.375560014, 3.021419373, 30.8046464, -1.918192848, -1.207302092, 19.50569021, 0.4846620972;
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
