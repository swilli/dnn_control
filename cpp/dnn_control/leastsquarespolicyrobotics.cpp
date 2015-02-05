#include "leastsquarespolicyrobotics.h"
#include "vector.h"
#include "lspisimulator.h"
#include "filewriter.h"
#include "configuration.h"

#include <cfloat>
#include <eigen3/Eigen/Dense>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <iomanip>


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

static LSPIState Sys2LSPI(const SystemState &state, const Vector3D &target_position) {
    LSPIState lspi_state;

    const Vector3D position = {state[0], state[1], state[2]};
    const Vector3D velocity = {state[3], state[4], state[5]};

    for (unsigned int k = 0; k < 3; ++k) {
        lspi_state[k] = target_position[k] - position[k];
        lspi_state[3+k] = -velocity[k];
    }

    return lspi_state;
}

SystemState InitializeState(SampleFactory &sample_factory, const Vector3D &target_position) {
    SystemState result;
    Vector3D position;
    for (unsigned int i = 0; i < 3; ++i) {
        position[i] = sample_factory.SampleUniform(target_position[i] - 3.0, target_position[i] + 3.0);
    }
    for (unsigned int i = 0; i < 3; ++i) {
        result[i] = position[i];
        result[i+3] = sample_factory.SampleUniform(-0.3, 0.3);
    }
    result[6] = sample_factory.SampleUniform(450.0, 550.0);
    return result;
}

std::vector<Sample> PrepareSamples(const unsigned int &seed, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_samples, const unsigned int &num_steps) {
    std::vector<Sample> samples;

    const double dt = simulator.ControlFrequency();
    SampleFactory sample_factory(seed);
    for (unsigned int i = 0; i < num_samples; ++i) {
        SystemState state = InitializeState(sample_factory, target_position);
        double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);

        for (unsigned int j = 0; j < num_steps; ++j) {
            const Vector3D position = {state[0], state[1], state[2]};
            const LSPIState lspi_state = Sys2LSPI(state, target_position);

            const unsigned int a = sample_factory.SampleRandomInteger() % kSpacecraftNumActions;
            const Vector3D &thrust = spacecraft_actions[a];
            const boost::tuple<SystemState, bool> result = simulator.NextState(state, time, thrust);
            const bool exception = boost::get<1>(result);
            if (exception) {
                break;
            }
            SystemState next_state = boost::get<0>(result);

            const Vector3D next_position = {next_state[0], next_state[1], next_state[2]};

            const LSPIState next_lspi_state = Sys2LSPI(next_state, target_position);


            const double error_state = VectorNorm(VectorSub(target_position, position));
            const double error_next_state = VectorNorm(VectorSub(target_position, next_position));
            const double r = error_state - error_next_state;

            samples.push_back(boost::make_tuple(lspi_state, a, r, next_lspi_state));

            time += dt;
            state = next_state;
        }
    }
    return samples;
}


static boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > TestPolicy(const unsigned int &seed, const eigen::VectorXd &weights, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_heights;

    Asteroid &asteroid = simulator.AsteroidOfSystem();
    const double dt = simulator.ControlFrequency();

    SampleFactory sample_factory(seed);

    SystemState state = InitializeState(sample_factory, target_position);
    double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);


    for (unsigned int i = 0; i < num_steps; ++i) {
        const Vector3D position = {state[0], state[1], state[2]};
        const Vector3D surface_point = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
        const Vector3D height = {position[0] - surface_point[0], position[1] - surface_point[1], position[2] - surface_point[2]};

        evaluated_positions.push_back(position);
        evaluated_heights.push_back(height);

        const LSPIState lspi_state = Sys2LSPI(state, target_position);

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

    return boost::make_tuple(evaluated_positions, evaluated_heights);
}

void TestLeastSquaresPolicyController(const unsigned int &random_seed) {
    Init();

    const double test_time = 24.0 * 60.0 * 60.0;

    LSPISimulator simulator(random_seed);
    SampleFactory &sample_factory = simulator.GetSampleFactory();
    const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights << -0.313304689, -0.01030083092, 0.01485461107, 0.001639202916, -0.002134176496, -0.3192744648, -0.01836197318, 0.006720414227, -0.01472152476, 0.01456222708, -0.0006674693636, -0.01709150426, -0.3023551701, 0.01719594304, -0.00677901472, 0.01696675825, 0.01290960053, -0.00036870298, -0.3295943347, -0.2315249179, -2.489519517, 0.2562475593, -0.173561103, -0.2453105648, -2.580949052, -0.01985984383, -0.2359656752, -2.34436586, -0.3066450007, -0.01030266504, 0.01481035197, 0.001308929999, -0.002357316704, -0.3131574718, -0.01306832905, 0.01752008741, -0.01548233231, 0.01444870112, -0.0003619987437, -0.005163376614, -0.2965480418, 0.01449888505, 0.006752757937, 0.0169456294, 0.02025090326, -0.007019655268, -0.3309303613, -0.2228448741, -2.574675037, 0.04259784334, -0.5467416622, -0.2373068429, -2.639231824, -0.007165187786, -0.03004980736, -2.332828594, -0.3034843383, -0.01170203478, 0.01445186459, 0.001423237172, -0.002417399053, -0.2986669728, -0.01843941595, 0.01833197831, -0.01637490931, 0.01448741296, -0.0002491215422, -0.007698829817, -0.2966217255, 0.006756006854, 0.0200635748, 0.01690247417, 0.01992833252, -0.01302383302, -0.3277113638, -0.1942472424, -2.720160417, 0.1282706384, -0.5155561165, -0.2373470651, -2.672129217, 0.3016913437, 0.1297891425, -2.384709237, -0.3074487145, -0.009473597322, 0.01467698886, 0.001485093688, -0.002247740836, -0.3103725994, -0.01768510998, 0.005631804647, -0.003611721149, 0.01465556176, 0.0001283455042, -0.009243674503, -0.3050589441, 7.169177431e-06, -0.006394243794, 0.01691321406, 0.01746803719, -0.01856004287, -0.3252580864, -0.2368217175, -2.560963393, 0.1890777588, -0.1772297781, -0.01866960291, -2.576214053, 0.3629587218, -0.2399545847, -2.487380777, -0.3096562589, -0.01105641771, 0.01506368679, 0.001679524186, -0.00147351565, -0.3268049087, -0.020888178, -0.00477893108, -0.004955318917, 0.01472424477, -0.0001979303276, -0.01590865163, -0.3103446481, 0.009965719317, 0.00713358443, 0.01692347783, -0.002162677673, -0.01229396031, -0.3250529635, -0.1949748029, -2.410560873, 0.3010502574, 0.08095296847, 0.002697923989, -2.477934716, 0.1646886658, -0.06366601762, -2.442711192, -0.3082768123, -0.01066159389, 0.01487782733, 0.001289222175, -0.002524099968, -0.3199233895, -0.009149450399, 0.02296708481, -0.004700824476, 0.01469828359, -0.0005749495435, -0.004040990914, -0.3081264875, 0.01636117906, 0.02026328759, 0.01679405352, 0.02568633391, -0.006283530548, -0.3220444749, -0.2121858044, -2.470849644, -0.02033422262, -0.6748766918, -0.01962086809, -2.520352117, 0.05811733456, 0.1177679928, -2.39576369, -0.3073794176, -0.008493225013, 0.01474612656, 0.00171478674, -0.002035805951, -0.3147959555, -0.02448833057, 0.004314971948, 0.009549579277, 0.01445781449, -0.0003337873232, -0.01418288244, -0.2977832524, 0.01860633798, -0.006734264276, 0.01682381519, 0.01200612964, -0.005520740984, -0.320945185, -0.2496610997, -2.520111085, 0.3135478462, -0.1307003055, 0.1609402168, -2.607593361, -0.09751960284, -0.2297252483, -2.543047702, -0.3097481164, -0.01062702875, 0.01490740487, 0.001140344039, -0.001962904276, -0.3216324094, -0.007819536555, 0.001944348557, 0.0080124177, 0.01447712414, 0.000199784283, -0.00141437002, -0.2951283382, -0.004546939048, 0.007498272236, 0.01707940915, 0.007580105966, -0.01986143646, -0.3337251972, -0.2096144249, -2.43982625, 0.02796639136, -0.08146615316, 0.1772280689, -2.663896592, 0.4378328578, -0.07213361629, -2.328507995, -0.3116610499, -0.01027335791, 0.01487232364, 0.001218626642, -0.002490532757, -0.3218196975, -0.01432378007, 0.01614244325, 0.007283479742, 0.01467135583, -0.0001220169705, -0.00217031959, -0.308810003, 0.004612890695, 0.02187113737, 0.01745059124, 0.02286797284, -0.01258622937, -0.3518077766, -0.2184549728, -2.453912248, 0.07315120375, -0.472775822, 0.1807798684, -2.510791943, 0.2325624805, 0.08252985259, -2.08790812, -0.3071499886, 0.001257172137, 0.01477012689, 0.001523225088, -0.002351617546, -0.3130495482, -0.01685225748, 0.01028056216, -0.01410013647, 0.01449955447, -0.0002775398594, -0.01249354245, -0.2987923056, 0.00872982434, -0.007182908265, 0.01680728905, 0.02119326874, -0.01056002289, -0.3250571958, -0.01481820478, -2.54080826, 0.2005611573, -0.3478037903, -0.2565944821, -2.658423734, 0.1260964162, -0.2236860675, -2.376959452, -0.3044139433, 0.001217032877, 0.01490435847, 0.001740100122, -0.002468034885, -0.3190591484, -0.0222048091, 0.01680511613, -0.01469777717, 0.01453769891, -0.0005181995425, -0.01686734071, -0.3025695754, 0.01478756048, 0.005097099553, 0.01695219737, 0.02164371379, -0.004811686131, -0.3296886158, -0.002287131884, -2.516700132, 0.2250001636, -0.457305056, -0.2479437574, -2.559098328, 0.007149721376, -0.02812604576, -2.347396089, -0.3001042359, 0.001540786018, 0.01464009435, 0.001700850222, -0.002351103069, -0.3043605454, -0.02110501357, 0.01509159429, -0.01630929496, 0.01445825206, -0.0001262127735, -0.01779527961, -0.2919946104, 0.007440292809, 0.02043197504, 0.01685304027, 0.02185213695, -0.01425371193, -0.3236221038, -0.008968274039, -2.743813945, 0.3653126696, -0.4987175867, -0.2149666038, -2.810132401, 0.1583354072, 0.1231661155, -2.434815367, -0.3035573405, 0.003571822197, 0.01470568844, 0.001529687536, -0.002002692077, -0.3102573523, -0.02040773129, 0.007364332689, -0.002982059158, 0.01477846247, -0.0003303613722, -0.0134954582, -0.3161402272, 0.009997243985, -0.006781947531, 0.01699281983, 0.01029730424, -0.01166386679, -0.3265181743, -0.04962626417, -2.626789869, 0.2993572296, -0.259327328, -0.03517545648, -2.393648473, 0.1855058122, -0.2275657833, -2.441488676, -0.3032077936, 0.002068598961, 0.01513444028, 0.001596703898, -0.002319005499, -0.3268854281, -0.01917449663, 0.01440521949, -0.003680475855, 0.01449184079, -0.0002404028901, -0.01013275623, -0.3003906659, 0.006237748338, 0.005688540832, 0.01711923514, 0.01664555637, -0.01479312713, -0.3374657057, -0.01120302062, -2.467082928, 0.2280645503, -0.3112231205, -0.03374874732, -2.574081493, 0.41626755, -0.02585929592, -2.266850856, -0.3027352738, 0.001735706169, 0.01475823054, 0.001812616708, -0.002229204701, -0.3104011013, -0.02569475798, 0.01351477736, -0.00393306172, 0.01453535915, -0.0001517268679, -0.01708079182, -0.2999060967, 0.006848326589, 0.01945550393, 0.01680256375, 0.0162003143, -0.01256661106, -0.3222549583, -0.0004900724607, -2.637058791, 0.4047580895, -0.3707840421, -0.02674166849, -2.602824373, 0.2681077034, 0.1423894627, -2.400156171, -0.3026938883, 0.003387245021, 0.01460866498, 0.001353793467, -0.002302030139, -0.3092924364, -0.017436374, 0.01240892153, 0.009139494684, 0.01449696323, -0.0005072370464, -0.003790873753, -0.3003269388, 0.0163380619, -0.006040146439, 0.0169976241, 0.01671023067, -0.006104168369, -0.333969554, -0.03440830712, -2.638401251, 0.1590643769, -0.3300515402, 0.1605456031, -2.576406459, -0.1036936103, -0.2429953232, -2.299018412, -0.3075523599, 0.003164324407, 0.01488514343, 0.001755141087, -0.002311657873, -0.3178861212, -0.02596061387, 0.01245116901, 0.008585125175, 0.01460502766, -0.0007552874886, -0.01494061944, -0.3014567495, 0.02214930475, 0.00679343321, 0.01695101752, 0.02274792641, 0.002890618921, -0.3311172606, -0.03075568052, -2.49204077, 0.3545496153, -0.4258294337, 0.1733318385, -2.63306782, -0.1293803367, -0.05143306826, -2.265549093, -0.3009122146, 0.002572874019, 0.01474631966, 0.001665813595, -0.002306301859, -0.3092534967, -0.02307964114, 0.01224076509, 0.008695656512, 0.01474086363, 9.514242253e-05, -0.01566001354, -0.310735147, 0.001729081232, 0.02098515365, 0.01700547424, 0.01287220155, -0.02042968306, -0.3299763286, -0.01037238819, -2.686575944, 0.2967060051, -0.1793020208, 0.1425674679, -2.480887467, 0.3263841308, 0.1152631366, -2.354822162, -0.3122329027, 0.01436377621, 0.01459485095, 0.001611730008, -0.001928849683, -0.3107904224, -0.02218159335, 0.002098293614, -0.01519143756, 0.01480536808, -0.0005013706132, -0.01259585531, -0.3144006543, 0.01024180419, -0.007322246475, 0.01698133236, 0.008404492987, -0.0009827681552, -0.3280456642, 0.1939564343, -2.526432822, 0.2946533473, -0.1089769004, -0.2223784288, -2.442011613, 0.03046841107, -0.218743626, -2.436370595, -0.3073887813, 0.01471579572, 0.01525478061, 0.00185041778, -0.001843719779, -0.337544314, -0.02760298231, 0.008270828427, -0.01546915828, 0.01479806837, -0.0002423040906, -0.01843424541, -0.3113864649, 0.01084632391, 0.004820788764, 0.01679282909, 0.004745289135, -0.01095652717, -0.3157126019, 0.1671661831, -2.326313285, 0.3880506742, -0.1454400785, -0.2502866472, -2.468219404, 0.109402607, -0.02531403013, -2.64256093, -0.3101298633, 0.01349498099, 0.014992655, 0.001823993542, -0.002532449366, -0.3210973812, -0.0268492595, 0.01692722472, -0.0151486516, 0.01456711183, -0.0005461154551, -0.02249917723, -0.3068977794, 0.01505016227, 0.01880733739, 0.01726574545, 0.02593056184, -0.001207626022, -0.3402409524, 0.2098846485, -2.547893511, 0.4629624923, -0.5393749743, -0.2461565445, -2.53763973, -0.07319935615, 0.1464682423, -2.354601306, -0.3013702727, 0.01481351783, 0.01465414675, 0.001627450612, -0.001935010307, -0.3096897099, -0.02135099718, -0.0004957774888, -0.002579344559, 0.01446794125, -0.0004472227987, -0.01204621252, -0.2963411485, 0.01139837453, -0.007989972248, 0.01699405721, 0.008190400102, -0.01034659148, -0.3304104158, 0.181948147, -2.620525059, 0.2660132963, 0.0575310919, -0.0366864063, -2.684693735, 0.2244713995, -0.2224266814, -2.36018502, -0.2995813292, 0.0145186621, 0.01489899108, 0.001405037471, -0.002079364885, -0.3173551976, -0.01666826111, 0.006216781816, -0.002653850512, 0.01439108749, -0.0003436891438, -0.007356823288, -0.2990005157, 0.0104168925, 0.005596521556, 0.01677135045, 0.009653825957, -0.005405657731, -0.3211410668, 0.2002402499, -2.528266745, 0.1693163996, -0.1351069443, -0.03999258505, -2.546781386, 0.04285762812, -0.02460091416, -2.457185605, -0.3088598514, 0.01658065494, 0.01504586154, 0.001568697044, -0.002196257631, -0.3280235622, -0.02155356718, 0.0116377489, -0.004139007856, 0.01479139725, -0.0005461092029, -0.011651929, -0.3113501649, 0.01654657527, 0.0182266133, 0.01718886444, 0.01461670981, -0.001863761177, -0.3372235034, 0.1483011247, -2.367647517, 0.2783139325, -0.2598808527, -0.02309533409, -2.520615362, -0.03192873612, 0.1651974333, -2.33532764, -0.3077324278, 0.01521529882, 0.01465158573, 0.001670585447, -0.002367518838, -0.3080281987, -0.02015337085, 0.014577593, 0.009212286451, 0.01465136462, -0.0001853085483, -0.01546215771, -0.3027666995, 0.007344989933, -0.006939292904, 0.01723283953, 0.02254040215, -0.009834587119, -0.3441823346, 0.1942821759, -2.690166718, 0.2723026769, -0.4906465883, 0.1759400125, -2.621602111, 0.1554725814, -0.2317555796, -2.178518288, -0.3072513855, 0.01624671382, 0.0148249952, 0.00167274824, -0.002697136443, -0.3184937096, -0.02521745313, 0.02545825038, 0.009421172906, 0.01473365684, -0.0002350837996, -0.01519358157, -0.3136464732, 0.005117557672, 0.006144767437, 0.01678080021, 0.02523899163, -0.01010278478, -0.3214587445, 0.162397199, -2.49462124, 0.3470042043, -0.6376732003, 0.1438324394, -2.368900993, 0.2179811049, -0.04340245552, -2.425243453, -0.2994438631, 0.01551663891, 0.01465801608, 0.001356392397, -0.002245708332, -0.3084679485, -0.01324873265, 0.008670111242, 0.009811541354, 0.01439711219, -0.0004285718903, -0.00973342479, -0.2914334342, 0.01238508095, 0.01861494034, 0.01699469556, 0.01627840966, -0.007854076071, -0.3285336407, 0.1636893659, -2.651692369, 0.1484975779, -0.2063493552, 0.1325255835, -2.759563124, 0.09476992092, 0.1645886713, -2.443505912;


    const unsigned int test_seed = sample_factory.SampleRandomInteger();
    std::cout << "testing policy ... ";
    const boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > result = TestPolicy(test_seed, weights, simulator, target_position, test_time / simulator.ControlFrequency());
    std::cout << "done." << std::endl << "writing result to file ... ";

    const std::vector<Vector3D> &positions = boost::get<0>(result);
    const std::vector<Vector3D> &heights = boost::get<1>(result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_LSPI_VISUALIZATION_FILE, simulator.ControlFrequency(), simulator.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;
}

void TrainLeastSquaresPolicyController(const unsigned int &random_seed) {
    std::cout << "LSPI seed: " << random_seed << std::endl;

    Init();

    const unsigned int num_samples = 1000;
    const unsigned int num_steps = 50;

    const double gamma = 0.9;
    const double epsilon = 1e-10;

    LSPISimulator simulator(random_seed);
    SampleFactory &sample_factory = simulator.GetSampleFactory();
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
