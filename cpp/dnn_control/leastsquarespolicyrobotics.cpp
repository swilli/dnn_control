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

static std::map<int, Vector3D> kSpacecraftActions;

static void Init() {
    unsigned int index = 0;
    const Vector3D actions_1d = {-kSpacecraftMaximumThrust, 0.0, kSpacecraftMaximumThrust};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                const Vector3D action = {actions_1d[i], actions_1d[j], actions_1d[k]};
                kSpacecraftActions[index++] = action;
            }
        }
    }
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

SystemState InitializeState(SampleFactory &sample_factory, const Vector3D &target_position, const bool &position_offset_enabled) {
    SystemState system_state;
    Vector3D position = target_position;
    if (position_offset_enabled) {
        for (unsigned int i = 0; i < 3; ++i) {
            position[i] += sample_factory.SampleUniform(- 3.0, 3.0);
        }
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
        const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
#endif
        const double dt = 1.0 / simulator.ControlFrequency();

        SystemState state = InitializeState(sample_factory, target_position, true);
        double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);

        for (unsigned int j = 0; j < num_steps; ++j) {
            const Vector3D position = {state[0], state[1], state[2]};
            const LSPIState lspi_state = SystemStateToLSPIState(state, target_position);

            const unsigned int a = sample_factory.SampleRandomInteger() % kSpacecraftNumActions;
            const Vector3D &thrust = kSpacecraftActions[a];
            const boost::tuple<SystemState, bool> result = simulator.NextState(state, time, thrust);
            const bool exception = boost::get<1>(result);
            if (exception) {
                break;
            }
            SystemState next_state = boost::get<0>(result);

            const Vector3D next_position = {next_state[0], next_state[1], next_state[2]};

            const LSPIState next_lspi_state = SystemStateToLSPIState(next_state, target_position);


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


static boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > TestPolicy(SampleFactory &sample_factory, const eigen::VectorXd &weights, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_heights;

    Asteroid &asteroid = simulator.AsteroidOfSystem();
    const double dt = 1.0 / simulator.ControlFrequency();

    SystemState state = InitializeState(sample_factory, target_position, LSPR_IC_POSITION_OFFSET_ENABLED);
    state[6] = simulator.SpacecraftMaximumMass();
    double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);

    for (unsigned int i = 0; i < num_steps; ++i) {
        const Vector3D position = {state[0], state[1], state[2]};
        const Vector3D surface_point = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
        const Vector3D height = {position[0] - surface_point[0], position[1] - surface_point[1], position[2] - surface_point[2]};

        evaluated_positions.push_back(position);
        evaluated_heights.push_back(height);

        const LSPIState lspi_state = SystemStateToLSPIState(state, target_position);

        const Vector3D thrust = kSpacecraftActions[Pi(sample_factory, lspi_state, weights)];
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
    SampleFactory &sample_factory = simulator.SampleFactoryOfSystem();
    const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights << -0.3275662742, -0.01838430727, 0.01393468929, 0.0004470929794, 0.0002504615373, -0.2979791091, 0.005977195315, -0.01254147938, -0.01208449068, 0.01513617018, 0.0007700896151, -0.009114240679, -0.313581802, 0.001729637842, -0.01111786364, 0.01568634075, 0.01923818832, -0.01384469504, -0.3088086952, -0.1224405066, -2.83712975, 0.2583384423, -0.04569681293, -0.2129931412, -2.641842253, 0.002646367402, -0.1764165325, -2.52874669, -0.3284242804, -0.01831383768, 0.01384245841, 0.0006527775238, 3.526841913e-05, -0.2910632118, 0.006970372235, -0.00999747551, -0.01393794753, 0.01522228286, 0.0006423328093, -0.01861064454, -0.3116961439, 0.007470366809, 0.002809039204, 0.01549965846, 0.0243174195, -0.01049575703, -0.299311903, -0.1052933443, -2.911972436, 0.3596676975, -0.08816464288, -0.1723539621, -2.720357363, -0.2566038097, -0.008637647651, -2.583111872, -0.3358463333, -0.0176965305, 0.01402691792, 0.0002596006302, -2.731260145e-05, -0.2982994073, 0.01352243088, -0.01225268702, -0.01294926107, 0.0153030453, 0.0008105751601, -0.00183999529, -0.3149725219, 0.0062976023, 0.01453152726, 0.01592070989, 0.02767649892, -0.01662672114, -0.3188546125, -0.1006404122, -2.870841745, 0.04543505542, -0.04166227653, -0.1953682853, -2.662407964, -0.0685147756, 0.2109369896, -2.403529188, -0.328692189, -0.0186938412, 0.01398903064, 0.0002874096018, 8.110996054e-05, -0.2985264948, 0.01510491376, -0.01571889862, -0.0009476398216, 0.01531222575, 0.001091137265, -0.006546918497, -0.3142305322, -0.004626202539, -0.01044117471, 0.0157837732, 0.02571286039, -0.01868343461, -0.3165348364, -0.1184489896, -2.846339055, 0.1341393876, 0.01424189556, 0.03422730756, -2.687511084, -0.08863728975, -0.1797633961, -2.400824826, -0.3252137293, -0.01734743754, 0.01397970731, 0.0003989927351, 0.0001713237404, -0.2930641547, 0.01187544461, -0.01751269634, -0.001223493762, 0.01512122055, 0.0006785534586, -0.01127416928, -0.3091871588, 0.01232700359, 0.002897359011, 0.01579009425, 0.02476430602, -0.01201347662, -0.3110278555, -0.1484205152, -2.956417735, 0.2875562662, 0.06119596051, 0.03667269851, -2.712502581, -0.2267750385, 0.005223064717, -2.507476316, -0.3223090271, -0.01761105479, 0.01377363029, 0.0005972869058, 0.0001200346735, -0.2920811336, 0.003819290505, -0.01588983582, -0.000161455431, 0.01504584358, 0.000720760291, -0.0142493089, -0.3054861198, 0.006975595432, 0.01431197153, 0.01581271434, 0.02354497163, -0.01428844896, -0.3176561924, -0.1262943882, -2.897885937, 0.3731075719, -0.01264111432, 0.008421540288, -2.739067407, -0.1617946195, 0.2098809945, -2.408710244, -0.3376916209, -0.01808344672, 0.01389541654, 0.0002540329894, -5.70009893e-05, -0.2954169532, 0.01267734588, -0.0121306076, 0.01105542565, 0.01522822966, 0.0003228460329, -0.004103785981, -0.3164947859, 0.02007088073, -0.009685338414, 0.01561394583, 0.02788929919, 0.002965474424, -0.3034174186, -0.148865756, -2.832425531, 0.142513405, -0.0463630788, 0.2288501859, -2.59359939, -0.540691959, -0.1766307074, -2.541190626, -0.3347598494, -0.01750810676, 0.01398696348, 0.0006374582083, 0.0001718915339, -0.297578556, 0.00496318413, -0.01557982624, 0.01078428831, 0.01538136328, 0.0006259615992, -0.01472003924, -0.3184952842, 0.0089760316, 0.002962715028, 0.01583571872, 0.02129491428, -0.0115783619, -0.3147043585, -0.1493738153, -2.862745344, 0.3847782903, 0.0834546051, 0.2607461763, -2.6340614, -0.1593223906, 0.01657583016, -2.472187582, -0.3314558285, -0.01807326911, 0.0138029697, 4.843910083e-05, 0.0001869123803, -0.2890070169, 0.02537313977, -0.01453083033, 0.01215348406, 0.01524613367, 0.0005846530419, -0.003892215146, -0.3159178075, 0.006839038006, 0.01582760074, 0.01563970244, 0.01955793275, -0.01357450011, -0.3057860074, -0.1175535794, -2.972955926, -0.08228865408, 0.01765879057, 0.2183230077, -2.56252801, -0.1234537228, 0.1992742109, -2.582728469, -0.333500426, -0.006456340894, 0.01395620792, 0.0006264245835, 0.0001148483488, -0.2973606896, 0.008865193284, -0.01654862945, -0.01186747874, 0.01520664011, 0.000747761703, -0.01068159221, -0.3115098014, 0.003626994054, -0.01058664039, 0.01587609809, 0.02893264515, -0.01026245487, -0.317368761, 0.1080759787, -2.858086179, 0.1748295338, -0.04648127388, -0.2219942932, -2.700484605, -0.220757015, -0.2074273787, -2.428561082, -0.3216462652, -0.007490849248, 0.01384219914, 6.66541715e-05, 0.0003898140664, -0.288317006, 0.02282585268, -0.02051447785, -0.01347251443, 0.01484736198, 0.0007121439271, -0.00289541654, -0.2934703689, 0.006065087357, 0.002083263354, 0.01569340803, 0.01797379515, -0.01619214202, -0.3101937538, 0.130932156, -2.967689198, -0.01220971137, 0.1047965609, -0.1740422381, -2.938123764, -0.07007356782, -0.008200275989, -2.474393015, -0.3350793391, -0.005500528958, 0.013936697, 0.0002971524018, 0.0002980792033, -0.2943061262, 0.01784467455, -0.01849580747, -0.01437224708, 0.0154681395, 0.0002278807964, -0.009387323963, -0.3191174779, 0.01913000471, 0.01315474145, 0.01553349715, 0.01721845821, -0.002033683627, -0.3044146412, 0.09919294332, -2.897486801, 0.09985793293, 0.1197416987, -0.1617527634, -2.648612263, -0.4582338748, 0.2107449354, -2.517208927, -0.333559269, -0.007365928364, 0.01388845854, 0.0004390914193, 0.000172876794, -0.2947539991, 0.01316855116, -0.01909724488, 0.0005405581455, 0.01509137549, 0.0008652184442, -0.009209110727, -0.3057378183, -0.003435200812, -0.009315467624, 0.01562864291, 0.02406198989, -0.01520240794, -0.3095593757, 0.1024052054, -2.833269097, 0.1352641593, 0.08989221837, 0.006708672848, -2.747804387, 0.05183399693, -0.2260755318, -2.422181935, -0.3194981418, -0.006472698073, 0.01383712088, 6.547505055e-05, 2.12183984e-05, -0.2920365591, 0.01875175656, -0.01113363648, -0.0005624889314, 0.01524372715, 0.00103459288, -4.434428459e-05, -0.3171143555, -0.003749153195, 0.001872030684, 0.01546077278, 0.0294070753, -0.02154484135, -0.2972421975, 0.09246231083, -2.926326526, -0.0151256097, -0.1530979842, 0.008832586719, -2.57975031, 0.07595535431, 0.00197974928, -2.664521142, -0.3303556644, -0.005606697217, 0.01408075643, 0.0004431152699, -9.909143729e-05, -0.2988382262, 0.01582808091, -0.007724771357, 0.0002808638783, 0.0151218931, 0.0009171541335, -0.009730685988, -0.3131659682, 0.001627971336, 0.01404452982, 0.01563277341, 0.03023132525, -0.01879736912, -0.302339794, 0.0967098892, -2.889845484, 0.01041216637, -0.1859554161, 0.003430436928, -2.535367573, 0.0007755863836, 0.2083034252, -2.618306067, -0.3261174603, -0.007545827205, 0.01398876278, 0.0001017148274, 9.565869787e-05, -0.2986664681, 0.01761509147, -0.01233404346, 0.01209237607, 0.01492016783, 0.0008760497839, -0.002037888028, -0.2974205266, -0.0003373812847, -0.01009618691, 0.0157159633, 0.02206923816, -0.01117415999, -0.3060463521, 0.1013117388, -2.863271638, 0.02012590945, 0.07590119681, 0.2161492104, -2.853533424, -0.1032268317, -0.1932035018, -2.632263331, -0.3245762101, -0.006955828282, 0.013932451, 0.0003752716695, 0.0002977495202, -0.2965036052, 0.01348181012, -0.01869533809, 0.01171623741, 0.01513657363, 0.0007422555135, -0.01009429289, -0.3104031172, 0.008239477617, 0.001406554717, 0.01568740154, 0.02080653734, -0.01717154467, -0.3082898896, 0.08841454638, -2.85871217, 0.1825424486, 0.04331737218, 0.2298008474, -2.720556982, -0.1499184895, 0.01862575648, -2.547152462, -0.3350965657, -0.006272172414, 0.0138332706, 0.0006318369107, 0.0002664042101, -0.2905030638, 0.005715344283, -0.01713535866, 0.01190718864, 0.0150791592, 0.000791741977, -0.01239569163, -0.3096814531, 0.00666856475, 0.01504639476, 0.01591998783, 0.02043642466, -0.01540826127, -0.3140859427, 0.0937974435, -2.95042211, 0.3159806866, 0.1299783111, 0.2345339575, -2.659297804, -0.195738446, 0.1880122464, -2.510348196, -0.3358544097, 0.004192620173, 0.01372112235, 0.0004176965876, 0.0004737088039, -0.2838478683, 0.01505134245, -0.02004731394, -0.01146204392, 0.01516856558, 0.0003609733113, -0.009921147117, -0.3111633794, 0.01578971247, -0.01169578851, 0.01555510984, 0.01777736244, -0.006614628467, -0.2991483892, 0.3521704087, -3.029728807, 0.1723029391, 0.1124974818, -0.2113900368, -2.67889472, -0.351791188, -0.196947642, -2.678502965, -0.3309668352, 0.005771265289, 0.01385577592, 0.0005223547597, -0.0001401340192, -0.2899079202, 0.00901540362, -0.008175223176, -0.01155514641, 0.0149538903, 0.0007370578867, -0.009551616522, -0.2991398523, 0.00262308982, -0.0007624803767, 0.01555770593, 0.03177651432, -0.01639557074, -0.2997222624, 0.330661134, -2.981903132, 0.1566730321, -0.2042926563, -0.2216797875, -2.805600033, -0.0828746649, 0.02140734293, -2.670522444, -0.3330198215, 0.005426147395, 0.01383966357, 0.0007017578727, 0.0008442064132, -0.2889404733, 0.004326442428, -0.03028904932, -0.0114741186, 0.0152014865, 0.0008365020046, -0.01393884426, -0.3098043533, 0.0006316590201, 0.01405502672, 0.01564604956, 0.005837231441, -0.01419999481, -0.3009124576, 0.339335516, -3.022741363, 0.3012623851, 0.4343500934, -0.2113191868, -2.715803772, 0.01999114967, 0.1688257822, -2.715037728, -0.3255528882, 0.004489493061, 0.01376147699, 0.0002672719939, -0.0001966011084, -0.2898956824, 0.01632540075, -0.006299627025, 0.0009114702953, 0.01499557801, 0.0006680671719, -0.01327830797, -0.3022988553, 0.009179926107, -0.01075320487, 0.01573813414, 0.03604989651, -0.01541023948, -0.3132188619, 0.3219374562, -2.963996387, 0.2146383463, -0.2282914983, -0.01793610473, -2.805797114, -0.06963075144, -0.2130460513, -2.504375194, -0.3330755291, 0.005984380919, 0.01380643801, 0.0005329873281, 0.0001375884471, -0.2922338094, 0.00840506803, -0.01361669909, 3.217886326e-05, 0.01539080252, 0.0007479005992, -0.01384476577, -0.3238112719, 0.003649081698, 0.002305922101, 0.01576840256, 0.02258508068, -0.01546280903, -0.3102057239, 0.3210632444, -2.855336484, 0.3195750914, 0.009233553444, 0.005935168924, -2.582800006, 0.0352180855, -0.02499403298, -2.536455125, -0.3356753363, 0.006369332242, 0.01371663268, 0.0002258372488, 0.0001447082393, -0.2857784024, 0.02004414118, -0.0161263024, -0.001174890162, 0.01508728241, 0.0008060570397, -0.008582072017, -0.307401585, -0.003282377785, 0.01299550984, 0.01594457254, 0.02187807961, -0.01486578969, -0.3202940027, 0.3062648492, -2.997276281, 0.1113060819, 0.02524765769, 0.03074049564, -2.779942584, -0.06722293887, 0.2036701679, -2.366984086, -0.334303163, 0.003510788161, 0.01398025515, 0.0004388843186, 0.0002452689013, -0.2978015323, 0.01033200703, -0.013569576, 0.0128687993, 0.01489081989, 0.0008545939957, -0.00938635163, -0.2960735863, 0.001836261569, -0.01026831115, 0.01583244373, 0.01827403116, -0.01888673901, -0.3204291524, 0.3381674575, -2.906729066, 0.2280119915, 0.03670062315, 0.2277415122, -2.906502629, -0.01226577921, -0.2093242389, -2.363463281, -0.3385280532, 0.004418893936, 0.01372536896, 0.000585269908, 0.0005027166156, -0.2872688832, 0.006658381084, -0.0211296833, 0.01244354586, 0.01532120482, 0.0009811901639, -0.01811520965, -0.3170644692, 0.002698671639, 0.002883604155, 0.01561419713, 0.01384090155, -0.0199016107, -0.3051455946, 0.3249605994, -2.91673439, 0.4685742828, 0.1964685661, 0.2229953057, -2.590998937, -0.05981379615, -0.008204002209, -2.590795674, -0.3383081403, 0.005186905875, 0.01370134754, 0.0002619104616, 6.849822793e-05, -0.2835085923, 0.01452724621, -0.01326605531, 0.01223021869, 0.01515699881, 0.0008412247044, -0.005530595252, -0.3064102693, 0.00859797455, 0.01471642609, 0.0154164793, 0.02910456467, -0.01669795301, -0.2970197509, 0.3181391388, -2.998412122, 0.1119330874, -0.1070981956, 0.2413012295, -2.737695544, -0.1970365015, 0.1924889454, -2.652954679;

    std::cout << "testing policy ... ";
    const boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > result = TestPolicy(sample_factory, weights, simulator, target_position, test_time * simulator.ControlFrequency());
    std::cout << "done." << std::endl << "writing result to file ... ";

    const std::vector<Vector3D> &positions = boost::get<0>(result);
    const std::vector<Vector3D> &heights = boost::get<1>(result);

    FileWriter writer(PATH_TO_LSPI_TRAJECTORY_FILE);
    writer.CreateVisualizationFile(simulator.ControlFrequency(), simulator.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;
}

void TrainLeastSquaresPolicyController() {
    Init();

    const unsigned int num_samples = LSPR_NUM_EPISODES;
    const unsigned int num_steps = LSPR_NUM_STEPS;

    const double gamma = LSPR_GAMMA;
    const double epsilon = LSPR_EPSILON;

    SampleFactory sample_factory(rand());

    const std::vector<Sample> samples = PrepareSamples(sample_factory, num_samples, num_steps);

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
