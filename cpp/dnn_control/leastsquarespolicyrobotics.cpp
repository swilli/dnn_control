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
        std::cout << std::endl << asctime(timeinfo) << "iteration " << ++iteration << ". Norm : " << val_norm << std::endl;
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

static std::vector<Sample> PrepareSamples(SampleFactory &sample_factory, const unsigned int &num_samples, const unsigned int &num_steps) {
    std::vector<Sample> samples;

    for (unsigned int i = 0; i < num_samples; ++i) {
        LSPISimulator simulator(sample_factory.SampleRandomInteger());
        const double dt = 1.0 / simulator.ControlFrequency();
        const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);

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


static boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > TestPolicy(SampleFactory &sample_factory, const eigen::VectorXd &weights, LSPISimulator &simulator, const Vector3D &target_position, const unsigned int &num_steps) {
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_heights;

    Asteroid &asteroid = simulator.AsteroidOfSystem();
    const double dt = simulator.ControlFrequency();


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
    SampleFactory &sample_factory = simulator.SampleFactoryOfSystem();
    const Vector3D spacecraft_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);
    Vector3D target_position;
    for (unsigned int i = 0; i < 3; ++i) {
        target_position[i] = spacecraft_position[i] + sample_factory.SampleUniform(-3.0, 3.0);
    }

    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights << -0.3661484382, -0.01203032968, 0.01399912564, -0.0004302560508, -5.042873925e-05, -0.2716689717, -0.01622416892, -0.004203989897, -0.01513792915, 0.01486299682, 0.0002920485787, 0.01092266946, -0.2934744494, -0.02454691929, -0.02424453281, 0.01669821832, -0.01529323118, 0.02239097702, -0.3232519585, -0.2039573485, -3.21341377, 0.09200080292, 0.1156909841, -0.2465729696, -2.699832125, -0.01347275115, -0.1165933419, -2.364642466, -0.3650994531, -0.01211815877, 0.01407418565, 4.531933754e-05, -0.0001460976555, -0.2768424715, -0.02417780875, 2.753081112e-05, -0.01271697031, 0.01469897449, 0.0001633554461, 0.01261712076, -0.2878121767, -0.01786275126, -0.009855794696, 0.01667243581, -0.01339067872, 0.01842655241, -0.3197991182, -0.2132720792, -3.095890831, -0.02104050615, 0.06414477425, -0.2776165726, -2.70295569, 0.02466134244, 0.06285045766, -2.45557765, -0.3675695091, -0.01215860675, 0.01381682805, 0.0003482052735, -0.0005498849569, -0.263712428, -0.03451819699, 0.007135852901, -0.01134572836, 0.01486555433, 6.659788214e-05, -0.002787093819, -0.2945400565, -0.01862620852, 0.001409562419, 0.01643207878, 0.0007555929295, 0.02665767943, -0.3100286853, -0.2026097327, -3.281598575, 0.3561420595, -0.2672256885, -0.2833347801, -2.655568384, -0.07633802311, 0.2745343661, -2.510546188, -0.3553479944, -0.01248506279, 0.01406265222, 0.0001369584184, -0.0002285588611, -0.2765932968, -0.02770074978, -0.001929847009, -0.002561195558, 0.01454498132, 0.0004093901912, 0.006966846601, -0.2787444805, -0.02709505723, -0.02390739813, 0.01643550539, -0.01280896765, 0.01767946012, -0.3123151912, -0.1766187305, -3.084825692, 0.1393767002, 0.1246933374, -0.05127315685, -2.90228039, 0.07200564019, -0.1321403019, -2.477258571, -0.3560972106, -0.0103494841, 0.01369899149, -0.0001518189972, -0.0005898828206, -0.256814749, -0.01849467558, 0.01009636149, -0.002174669694, 0.01487276485, 0.0003683554414, 0.01093143121, -0.2969284224, -0.02537334135, -0.01095525914, 0.01671422372, -0.003580520719, 0.01827823914, -0.3226115863, -0.2246026861, -3.368689553, -0.06529528741, -0.1549358586, -0.03308232173, -2.66268362, 0.07087583502, 0.04740219827, -2.377441921, -0.3701202884, -0.01145198175, 0.01420683799, 0.0003589072411, -0.0005285972905, -0.2821517598, -0.03462914612, 0.005903499023, -0.001125152493, 0.01467502956, 0.0004726919697, 0.001006127968, -0.2847183123, -0.02683125512, 0.001283059292, 0.01672110542, -0.001637922905, 0.01385114968, -0.3215860191, -0.1838411683, -3.044359863, 0.335844994, -0.2094049367, -0.04445901399, -2.780085466, 0.1077992908, 0.2557160066, -2.393365339, -0.3615109464, -0.009970205523, 0.0138473103, -0.0001547388619, -0.0002745963605, -0.263376811, -0.02511904125, 0.004599503221, 0.008872245398, 0.01466631028, 2.744319034e-05, 0.01495871913, -0.2842303731, -0.01841218724, -0.02524815527, 0.01643769764, -0.009437620024, 0.02598856906, -0.3128489413, -0.2204148954, -3.256863559, 0.04038410923, -0.09954590171, 0.1692378162, -2.784292915, -0.06344786949, -0.1222003234, -2.470051175, -0.3603120287, -0.009925916724, 0.01382626185, -0.0001214382601, -0.0007431173177, -0.2649048103, -0.02024506296, 0.01254465619, 0.01011330257, 0.01467338862, 0.0004396164079, 0.01147893008, -0.282943423, -0.02573440505, -0.01249468404, 0.0165132846, 0.002193684354, 0.01709473037, -0.3131617663, -0.2181187288, -3.246887114, 0.05441512479, -0.3533686725, 0.1702843799, -2.864319379, 0.05627941328, 0.07244776301, -2.441668653, -0.3663330586, -0.009611255792, 0.01402006011, 0.0001026879291, -0.0001970298366, -0.273100249, -0.02708104246, 0.0006910421993, 0.01109791025, 0.01487674316, 0.0001780638193, -0.0003241113135, -0.2963903661, -0.02426972304, 0.001489915346, 0.0167254187, -0.008326697741, 0.02086533105, -0.3255576205, -0.220880913, -3.181253863, 0.2588996222, -0.1046638009, 0.1547604416, -2.648842081, 0.05915520117, 0.2190866448, -2.316515029, -0.3609154622, -0.0009929394821, 0.01416837093, 0.0001174452321, -0.0004949355653, -0.2774820677, -0.0251506009, 0.006527835549, -0.01495451568, 0.01462935429, 0.0002691402728, 0.003428683812, -0.2825763562, -0.02492954035, -0.02227573365, 0.01624633222, -0.003168031562, 0.01592975585, -0.3015366831, 0.03771251278, -3.124351874, 0.2210603759, -0.2323238937, -0.2578537207, -2.843765473, 0.1011011605, -0.1501458411, -2.597030888, -0.3551841887, 2.256711662e-05, 0.01395120511, 0.000201606926, -0.000340459324, -0.2709533232, -0.028082129, 0.00219919405, -0.01266430412, 0.01487316137, 6.418585575e-05, -0.005223802131, -0.2966867349, -0.01977043762, -0.009802625832, 0.01640912692, -0.007413525495, 0.02562490527, -0.3065407148, 0.0136754694, -3.224031448, 0.3364591746, -0.02646917458, -0.2982057908, -2.627168881, -0.04494516319, 0.0471943808, -2.605610052, -0.3647242266, -0.001347184733, 0.01365799505, 3.90591536e-06, -0.0003884581317, -0.2591939898, -0.02586283177, 0.001487927701, -0.0123920526, 0.01454659337, 0.0002387201576, 0.007055980152, -0.2780838747, -0.02300420138, 0.003107472924, 0.01644883973, -0.004430240219, 0.01805942273, -0.3072795948, 0.05435844304, -3.291309036, 0.1632493431, -0.05697819383, -0.2893972036, -2.826509578, 0.07646244059, 0.2326149031, -2.542244354, -0.3563787012, -0.001184430517, 0.01390453945, 0.0001766170165, -0.0002697831141, -0.2629800743, -0.03137428862, 0.00356344444, -0.002858100833, 0.01469942609, 0.0003776916503, 0.005595678046, -0.2891673028, -0.02491021663, -0.02331032427, 0.01664375378, -0.00587821847, 0.02083299988, -0.3195004468, 0.06800677825, -3.357910461, 0.09203085976, -0.1026202269, -0.06579200414, -2.700590912, -0.02483335232, -0.1470082439, -2.394532871, -0.3579055894, 0.0001168013818, 0.01387755715, -9.586929545e-05, -0.0001974399867, -0.2728265974, -0.02806972553, 4.82693077e-06, -0.001374031525, 0.01470905996, 0.0001092818281, 0.01133381633, -0.2855125296, -0.01624576657, -0.01133388435, 0.01650008079, -0.01010607937, 0.02105982783, -0.3137950421, 0.03202138161, -3.092873554, 0.2311346916, -0.01965500567, -0.08562957479, -2.729811895, -0.03313742719, 0.084684648, -2.49694757, -0.3613006832, -0.0005761443017, 0.01416405379, 0.0001255127813, -0.0003711951598, -0.2788840794, -0.02631879208, 0.003792200188, 0.0005461280981, 0.01471335031, 0.0003288029022, 0.006497472955, -0.2860993461, -0.02455098486, 0.001993134211, 0.0165551156, -0.008029253907, 0.01294310501, -0.3133063085, 0.05967910931, -3.090013, 0.06380044924, -0.1567356101, -0.1010969416, -2.822782259, 0.1378466016, 0.2509755958, -2.52871788, -0.3542626423, 0.002044876071, 0.01388836384, -0.000169393626, -0.0004843026494, -0.2686114056, -0.02209990827, 0.008032458733, 0.00738245248, 0.01464325736, 8.374016515e-05, 0.009500246625, -0.279042704, -0.01552047956, -0.0238951745, 0.01655823189, -0.004051996992, 0.02729435814, -0.3181625547, 0.0007211299323, -3.198027617, 0.09845718233, -0.2251853905, 0.1768287131, -2.904549733, -0.196317817, -0.1350166681, -2.441789436, -0.3535836922, 0.001228198831, 0.0138005026, 0.0002312354279, -0.0005142532618, -0.2624012666, -0.03364463064, 0.01265617783, 0.01047600845, 0.01493403588, 0.0004164420568, 0.001590702281, -0.3026548866, -0.02714596965, -0.009884944311, 0.01661384982, 0.001543165661, 0.01949609093, -0.3219863747, 0.03820423199, -3.26954815, 0.2678735697, -0.3754783936, 0.1582528224, -2.582646312, 0.04704165922, 0.01282237396, -2.355108812, -0.3661883865, 4.260608643e-05, 0.01386801399, 0.000128004456, -5.1982073e-05, -0.2699804595, -0.03135047488, -0.005407604847, 0.01014953238, 0.01483288476, 0.0001533629068, 0.00322678629, -0.2950637003, -0.02003418107, -0.0001910819853, 0.01649265586, -0.01413780547, 0.02277630902, -0.3120251952, 0.03451200216, -3.161639253, 0.291161924, 0.0775475021, 0.1567399075, -2.606433862, -0.02553165434, 0.2620962164, -2.497779797, -0.3671284412, 0.009589754343, 0.01404288801, -0.0004277170245, -0.0006727520853, -0.2704402566, -0.01814309626, 0.01256388413, -0.01487191093, 0.01463758894, 0.000464587462, 0.01558620264, -0.2835874439, -0.02770045893, -0.02305228466, 0.01664655202, -0.002414132401, 0.01915518341, -0.3237608617, 0.2984718809, -3.262527521, -0.05606125905, -0.2338814249, -0.2732121887, -2.820144101, 0.0707784508, -0.1133834991, -2.349730666, -0.363224446, 0.008580611987, 0.01383874501, 0.0001416091288, -0.000449505198, -0.2619344687, -0.03004282242, 0.008427600939, -0.01443875053, 0.0148938886, 0.0002316968262, 0.008272704552, -0.2956866611, -0.01975071252, -0.008800437074, 0.01636852597, -0.004255523749, 0.01976125918, -0.3075346322, 0.3164780378, -3.338502207, 0.08257666764, -0.1759218879, -0.244219717, -2.668264897, -0.05107910563, 0.04322522658, -2.567064821, -0.3738640256, 0.008324838174, 0.01389692372, -0.0001220139431, -0.0002421730386, -0.2681009543, -0.02623038158, 0.002664072415, -0.01491374205, 0.01519524655, 0.0004020536711, 0.0120358995, -0.310080878, -0.02532733105, 0.002286516439, 0.01664387388, -0.010445884, 0.01472609176, -0.3206304267, 0.3241433609, -3.22411859, 0.08737908847, -0.04949443829, -0.2105209467, -2.533336104, 0.08067375098, 0.275964868, -2.409563843, -0.3574877836, 0.01133179476, 0.01404324916, -0.0005708651145, -0.0004394502916, -0.2751377902, -0.01482795775, 0.01078183343, -0.003454921705, 0.01464899567, 0.0001866323917, 0.02212766765, -0.2812562236, -0.01787799742, -0.02403631498, 0.01645092004, -0.008505346825, 0.02044934219, -0.3118768859, 0.2680895798, -3.089571618, -0.1745017841, -0.1934570346, -0.04845644584, -2.821373084, -0.09331103984, -0.1126355957, -2.529726657, -0.3568943301, 0.01114807988, 0.01382261914, 0.0001979393965, -0.0003565031393, -0.2633263588, -0.02792636702, 0.003952189696, -0.00160133296, 0.01476666794, 0.0006762479856, -0.002010961971, -0.2897693343, -0.03370229841, -0.01068504183, 0.01680611562, -0.008747247044, 0.009075372101, -0.3277988983, 0.287816808, -3.330303461, 0.2961985686, -0.1263119882, -0.06601133373, -2.74234649, 0.2550903727, 0.06782385881, -2.321411142, -0.3670038432, 0.01125493128, 0.01396508904, 0.0002447672728, -0.0004885859006, -0.2732854191, -0.03226868985, 0.00914458016, -0.001191202981, 0.01490772021, 0.000604912767, 0.00166390943, -0.3001797418, -0.03272154348, 0.0007714234186, 0.01673923195, -0.00105477271, 0.01268956134, -0.3250720129, 0.2776212809, -3.13854732, 0.1812137679, -0.3588823699, -0.071691046, -2.595526279, 0.1872381045, 0.2846025596, -2.385021545, -0.3608444982, 0.01172889273, 0.01407761458, 0.0002345992822, -0.000241301858, -0.276257555, -0.03773199025, 0.002799269653, 0.008926312719, 0.01470185339, 0.0001733399287, 0.0003889242124, -0.2850333233, -0.01846176858, -0.02408333286, 0.0165669394, -0.007564280292, 0.02238470526, -0.3184918723, 0.2885184482, -3.165789745, 0.4526508554, -0.08962256683, 0.1451033775, -2.801553105, -0.0676300163, -0.1340107946, -2.442992857, -0.3620409397, 0.01260584327, 0.01390593144, 0.0003443487115, -0.0004420950563, -0.2723115519, -0.03292794263, 0.01108087923, 0.009495495066, 0.01471849067, 0.0006558320031, 0.001556997103, -0.2837329233, -0.03024863777, -0.01046714672, 0.01643705854, -0.01169498612, 0.009557195388, -0.3153642405, 0.2571267546, -3.101055019, 0.1895486802, -0.04068627379, 0.1511640053, -2.824298952, 0.2269526316, 0.04460358374, -2.411597381, -0.3768752222, 0.01158901868, 0.013898809, 0.0002968376426, -0.0003117224062, -0.2723461685, -0.03195599027, 0.008212562412, 0.01022668982, 0.01502989839, 0.000437804921, -0.001829474212, -0.2978876652, -0.02423033318, 0.001212474372, 0.01682873107, -0.009268558982, 0.02009796733, -0.3312062456, 0.2760211864, -3.119129941, 0.2292326142, -0.1446228128, 0.1454514902, -2.65380482, -0.03657716585, 0.2606826856, -2.265180131;

    std::cout << "testing policy ... ";
    const boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > result = TestPolicy(sample_factory, weights, simulator, target_position, test_time / simulator.ControlFrequency());
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

    SampleFactory sample_factory(random_seed);

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
