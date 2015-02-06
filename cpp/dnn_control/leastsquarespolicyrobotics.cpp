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
            const LSPIState lspi_state = SystemStateToLSPIState(state, target_position);

            const unsigned int a = sample_factory.SampleRandomInteger() % kSpacecraftNumActions;
            const Vector3D &thrust = spacecraft_actions[a];
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


    SystemState state = InitializeState(sample_factory, target_position);
    double time = sample_factory.SampleUniform(0.0, 12.0 * 60.0 * 60.0);

    for (unsigned int i = 0; i < num_steps; ++i) {
        const Vector3D position = {state[0], state[1], state[2]};
        const Vector3D surface_point = boost::get<0>(asteroid.NearestPointOnSurfaceToPosition(position));
        const Vector3D height = {position[0] - surface_point[0], position[1] - surface_point[1], position[2] - surface_point[2]};

        evaluated_positions.push_back(position);
        evaluated_heights.push_back(height);

        const LSPIState lspi_state = SystemStateToLSPIState(state, target_position);

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
    weights << -0.342274913, -0.0144465702, 0.01507086566, 0.00129849301, 0.001357526076, -0.2969127667, 0.009405848943, -0.003974280148, -0.01639268788, 0.01444377447, -0.0005174022769, -0.02879064016, -0.281565641, 0.01356358461, -0.006795608775, 0.01567380099, -0.03045200005, 0.01749479802, -0.3249370113, -0.2499755202, -3.122695761, 0.1737269873, 0.3261826961, -0.211598594, -2.676338197, -0.2302613662, -0.2135903075, -2.32157234, -0.3262375982, -0.01381418422, 0.01511293059, 0.001819486473, 0.0007216800118, -0.3014549973, -0.004427331985, 0.01153521874, -0.01582709621, 0.01419256867, -0.0005322611277, -0.03957257271, -0.2732024189, 0.01465985306, 0.006143732249, 0.01530575432, -0.01012186837, 0.01473429232, -0.2999404387, -0.2758602926, -3.031849011, 0.3785376572, -0.1895355414, -0.2099505618, -2.799723611, -0.248018197, -0.02566742427, -2.668352418, -0.3284043035, -0.01415169533, 0.01527283834, 0.001328864336, 0.0005613901799, -0.3079172342, 0.00554637224, 0.01599678188, -0.01695365098, 0.01402438694, -0.000361283498, -0.02746005404, -0.2644561184, 0.007315450439, 0.01911101999, 0.01548298201, -0.006016034448, 0.01514382033, -0.3092011139, -0.2677211047, -2.983875954, 0.174741756, -0.2888571051, -0.1934372839, -2.863629407, -0.1576108065, 0.1556283834, -2.66033918, -0.3294766337, -0.0147188109, 0.01535413785, 0.001557125209, 0.0009296113418, -0.3146671956, 0.0003149448363, 0.005420031754, -0.004113700838, 0.0143941764, -0.0006562458202, -0.03528670918, -0.2796003207, 0.02531904931, -0.005827790876, 0.01562053969, -0.01547868474, 0.01550744226, -0.3166875714, -0.2655218659, -2.866161678, 0.2831878378, -0.05105547885, -0.01695261223, -2.775173146, -0.3539728875, -0.2437932653, -2.530003891, -0.327259612, -0.01392743537, 0.01523883913, 0.001367582564, 0.001068347227, -0.3051011515, 0.005831640431, 4.869291347e-05, -0.005910356087, 0.01437903767, -0.0003704149221, -0.02698552441, -0.2806120366, 0.01273457522, 0.004977721653, 0.01548074943, -0.02050011948, 0.01595697049, -0.310833906, -0.2611097871, -3.082874787, 0.1603166195, 0.03479060527, 0.02560279766, -2.64768346, -0.2915025968, -0.007706905973, -2.570706174, -0.3412028833, -0.01552851296, 0.01544231895, 0.001738715468, 0.0008684205583, -0.3132967003, -0.004112954914, 0.006260723191, -0.006191753967, 0.01457863046, -0.0008142141041, -0.03383044237, -0.2936357631, 0.02493336774, 0.01902196887, 0.01556587011, -0.01929126764, 0.01973439702, -0.3188752968, -0.2354056982, -2.897671131, 0.2493664258, 0.0838585827, 0.02475482826, -2.525435284, -0.4295990284, 0.131749974, -2.34595483, -0.3271514682, -0.01488464293, 0.01505656196, 0.001279501751, 0.000850525594, -0.2983258057, 0.009656954813, 0.00970211019, 0.005715259585, 0.01422868796, -0.0006141899316, -0.02850964814, -0.2685657915, 0.01521960308, -0.006011130289, 0.01553732401, -0.01625705933, 0.0209713709, -0.3109737438, -0.2825059116, -3.126458082, 0.02381909721, -0.06515340979, 0.2239502094, -2.896947185, -0.2514733079, -0.2276203491, -2.594827383, -0.3334531954, -0.01554675101, 0.01532606795, 0.00137974243, 0.0006269945636, -0.3119382749, 0.005476280603, 0.01081515916, 0.005854429941, 0.01415019794, -0.00054821739, -0.03129415628, -0.2718212017, 0.01291186441, 0.00442193247, 0.01555783347, -0.009529788027, 0.02039871881, -0.3164891373, -0.261343591, -2.944731114, 0.1914857924, -0.139176858, 0.2176363622, -2.729601822, -0.2682549323, -0.01129293614, -2.469511812, -0.3384118074, -0.01433728227, 0.01536435783, 0.001196625301, 0.0007513156199, -0.3107664588, 0.00862134337, 0.009781313888, 0.004666868281, 0.01435597383, -0.0006025769322, -0.02352094078, -0.2783213984, 0.01712062658, 0.0182122071, 0.0156391423, -0.0152964776, 0.01639824999, -0.320658059, -0.2907645344, -2.977727314, 0.04101516518, 0.02276473327, 0.249194593, -2.699123068, -0.2509040603, 0.1714987, -2.37895295, -0.3227295593, -0.002286949211, 0.01503818198, 0.001633410812, 0.0009807135582, -0.2977868504, -0.001360209967, 0.007320767153, -0.01439972514, 0.01435725873, -0.0006724199513, -0.03547290277, -0.2823719799, 0.01764968484, -0.004016164154, 0.01540901369, -0.01991183455, 0.02080562029, -0.303611648, -0.01871117879, -3.110650339, 0.2532542998, -0.02418018469, -0.2224417176, -2.689043112, -0.2498102878, -0.2581524121, -2.700197422, -0.3307605792, -0.002315599515, 0.0153206542, 0.001480462932, 0.0008974576599, -0.3133674119, -0.001271056102, 0.01128383599, -0.01498843253, 0.01473285658, -0.0001651638448, -0.02725006544, -0.2988460674, 0.003365008231, 0.008167415823, 0.01559309357, -0.01501706607, 0.007813731489, -0.31052696, -0.01640667827, -2.886405561, 0.196467895, -0.2120092777, -0.225272622, -2.486355583, 0.001662212056, -0.069127972, -2.639376677, -0.3288408168, -0.002900294559, 0.01561902944, 0.001765330246, 0.0007991518906, -0.3207923977, -0.002702915164, 0.009505735757, -0.01464688599, 0.01433528875, -0.0006104841202, -0.04019745412, -0.2803306333, 0.01691657585, 0.01850480579, 0.01550828649, -0.01122911044, 0.02135281154, -0.3090353359, -0.01862549249, -2.864400824, 0.3921518596, -0.1504445508, -0.2470492483, -2.692285317, -0.3370468824, 0.1919790276, -2.596987232, -0.317819162, -0.001467573705, 0.01486507381, 0.001670159808, 0.0007528347997, -0.2948096957, 0.0006734354898, 0.01292955329, -0.00392509637, 0.014296461, -0.0002528771779, -0.03664619828, -0.2722399048, 0.007823609364, -0.007115836138, 0.01546699133, -0.01575830526, 0.005656656636, -0.3120902341, -0.05593370142, -3.09561254, 0.3266071979, -0.1789563612, 0.001735430769, -2.856780744, 0.02395605171, -0.2043672532, -2.537857283, -0.324751175, -0.001699595646, 0.01536031063, 0.001095881085, 0.001036421937, -0.3129798392, 0.0143728997, 0.004405398259, -0.004142973516, 0.01432679956, -0.000379822935, -0.03076216466, -0.285100977, 0.01228546868, 0.005740381092, 0.01568037143, -0.02164351864, 0.007627682279, -0.3166875926, -0.04342772208, -2.921876074, 0.1539552713, 0.109320363, -0.003800242605, -2.583284258, -0.1047184337, 0.003683953108, -2.463795997, -0.3222334355, -0.002477444067, 0.01501818492, 0.001444821985, 0.0009886248443, -0.2948540791, 0.007343357895, 0.005936641745, -0.005821315508, 0.01432763485, -0.0004726155209, -0.03367773738, -0.2794835628, 0.01133748253, 0.01811643679, 0.0155411984, -0.01861335555, 0.01874831514, -0.3088562907, -0.03477488466, -3.129751795, 0.1725561021, 0.0007495375013, 0.03401999144, -2.646781702, -0.1989890789, 0.1933856275, -2.673517231, -0.3193648913, -0.00274539386, 0.01510866429, 0.001253198103, 0.0007927119075, -0.3020607973, 0.01340121986, 0.01071169494, 0.007665409284, 0.01419000038, -0.0005197916959, -0.02702413752, -0.2736212779, 0.01156877829, -0.00638407461, 0.01523691704, -0.01150521437, 0.01502861821, -0.3016860481, -0.04568258181, -3.085343872, 0.05662211962, -0.152343255, 0.2100447167, -2.781047063, -0.1696325918, -0.2208901642, -2.587175434, -0.3209887585, -0.003112745958, 0.01522186086, 0.001152839729, 0.000628812764, -0.3041269162, 0.01104420245, 0.01577674332, 0.006552119712, 0.01448460432, -0.0004933596938, -0.02460621867, -0.2879516655, 0.01592172047, 0.005380152784, 0.01545374461, -0.007792564023, 0.01975511616, -0.3080492423, -0.03805952712, -3.070171946, 0.01846397948, -0.2434165834, 0.2022538702, -2.594353903, -0.338241971, -0.006371123997, -2.645219443, -0.3255455236, -0.00309939948, 0.01476776315, 0.001447422875, 0.0008769599028, -0.2879596778, 0.00733143644, 0.008623586085, 0.00614634339, 0.01437219791, -0.0004946747399, -0.0244975135, -0.287965521, 0.01417068676, 0.01914717696, 0.015489022, -0.01432000279, 0.01525511817, -0.3102929149, -0.02551720797, -3.187323481, -0.05919541391, -0.05824591314, 0.2226341625, -2.567766497, -0.2236153449, 0.1576610307, -2.532077945, -0.3372802193, 0.009650070148, 0.01530936819, 0.001938029645, 0.001071348998, -0.3119074347, -0.006225155349, 0.001863360262, -0.01396988269, 0.01449767564, -6.888020419e-05, -0.04771639516, -0.2883343707, 0.006809279637, -0.00465894597, 0.01555698731, -0.0193174956, 0.001072442729, -0.3147184012, 0.2143921315, -2.934964627, 0.474223802, 0.1818047572, -0.2015577021, -2.615892477, 0.1248246775, -0.2028165938, -2.478655264, -0.3340520456, 0.01028763233, 0.01529842882, 0.00119863874, 0.001059142757, -0.3123738527, 0.008706611649, 0.00304757134, -0.01415242949, 0.01431425277, -0.0002825052438, -0.02328864783, -0.2784159941, 0.01233114044, 0.007349532772, 0.01582287239, -0.01756525418, 0.01155913301, -0.322525466, 0.2119082715, -2.88805422, 0.001186893392, -0.02194083429, -0.1992139017, -2.715508883, -0.1822291992, -0.02513524937, -2.441296393, -0.3243169223, 0.010581651, 0.01502031945, 0.001026351151, 0.0006797159144, -0.294847696, 0.01285158716, 0.0105894744, -0.01575636154, 0.01420583336, -0.0006925616576, -0.01972579634, -0.2753353845, 0.0190407312, 0.01993309457, 0.01565161112, -0.0107697426, 0.01657987677, -0.3166639598, 0.1776680098, -3.164399078, -0.04229586073, -0.1013953288, -0.1779659914, -2.768658111, -0.3012774843, 0.1683462521, -2.530008488, -0.3302090144, 0.01015877905, 0.01508497262, 0.001219309958, 0.0009017759983, -0.2995813973, 0.01304514188, 0.008388871663, -0.002839039532, 0.01462600492, -0.0003469189925, -0.02659705369, -0.292993136, 0.00761416168, -0.005277384747, 0.01533611171, -0.01813136506, 0.01611571016, -0.3063885514, 0.2009760095, -3.07276742, 0.02374003282, -0.002560833735, -0.006281870392, -2.486182265, -0.1807485619, -0.2191942185, -2.575240623, -0.3244978733, 0.009086697691, 0.01512403484, 0.001295703077, 0.001457954772, -0.3016447266, 0.004422555271, -0.007093544204, -0.002878381303, 0.01454123343, -0.0006952156137, -0.02663714089, -0.2898001334, 0.01952649158, 0.006664097002, 0.01580654551, -0.02841989932, 0.02321200531, -0.3307495195, 0.1990215978, -3.060274567, 0.1820656692, 0.2934148675, 0.004636885716, -2.581331677, -0.3698205573, -0.01223977791, -2.282058313, -0.3282661124, 0.008936917105, 0.01517762718, 0.001779614044, 0.00113283192, -0.3041613326, -0.00762557458, 0.006331097584, -0.003850329234, 0.01445947575, -0.0007077062658, -0.04109174245, -0.2871871514, 0.02083807682, 0.02037117887, 0.01549941192, -0.01826910691, 0.02459791104, -0.3147742769, 0.2147581252, -3.059856805, 0.4587658705, -0.09861551974, 0.008803100817, -2.547100362, -0.4912178532, 0.1629593463, -2.434890291, -0.3370983102, 0.009562857235, 0.01491010004, 0.001322675163, 0.0008729194711, -0.2963221396, 0.00599553464, 0.008695799983, 0.008132036465, 0.01433308866, -0.0008359218651, -0.02535728017, -0.2818495037, 0.0229773258, -0.005420400443, 0.01599874422, -0.01453085105, 0.02584751951, -0.3347318624, 0.2000160163, -3.092618872, 0.03978475307, -0.113473408, 0.198717474, -2.633836441, -0.4588538191, -0.2178160839, -2.200004713, -0.3294238891, 0.008545729129, 0.01509849561, 0.001676824505, 0.0007145711999, -0.3029764069, -0.003555481358, 0.01167530015, 0.007384401557, 0.01469913713, -0.0004849928089, -0.03489315027, -0.2954979184, 0.01162845234, 0.005406089066, 0.01562597866, -0.01155892395, 0.01724233655, -0.3146471129, 0.2236743202, -3.03655861, 0.4041867695, -0.2242248712, 0.2220109378, -2.525420434, -0.2367086163, 0.001120951382, -2.522833531, -0.3277153278, 0.00999407993, 0.01507778427, 0.001298995768, 0.000951106188, -0.3033668323, 0.006768886835, 0.006610032345, 0.006829419125, 0.01451298167, -0.0008354507944, -0.0304118029, -0.2823387978, 0.02284804389, 0.01981450616, 0.01554529287, -0.01580489385, 0.02752506492, -0.3155362584, 0.1687894796, -3.03259878, 0.1395225288, -0.09147338539, 0.2239657036, -2.717886035, -0.5017975171, 0.1757696252, -2.508511611;

    std::cout << "testing policy ... ";
    const boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > result = TestPolicy(sample_factory, weights, simulator, target_position, test_time * simulator.ControlFrequency());
    std::cout << "done." << std::endl << "writing result to file ... ";

    const std::vector<Vector3D> &positions = boost::get<0>(result);
    const std::vector<Vector3D> &heights = boost::get<1>(result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_LSPI_VISUALIZATION_FILE, simulator.ControlFrequency(), simulator.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;
}

void TrainLeastSquaresPolicyController() {
    Init();

    const unsigned int num_samples = LSPR_NUM_SAMPLES;
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
