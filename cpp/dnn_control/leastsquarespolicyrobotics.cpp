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
    weights << -0.367593561, -0.008280802155, 0.01329872552, -0.0007295507426, -0.001286729926, -0.2678216433, 0.006119936395, 0.004182264349, -0.01358438663, 0.01358683836, 0.002386365159, 0.02444087788, -0.2750316938, -0.02845175147, -0.01135993856, 0.0160325365, 0.03131913116, 0.003193451772, -0.3171350887, -0.2859609625, -3.091037994, -0.1192480655, -0.5193021794, -0.2105684122, -3.007468521, 0.1997002963, -0.1708166347, -2.38046837, -0.3587762512, -0.007817380911, 0.01333703349, -0.0002387522042, -0.001097329634, -0.2746580293, -0.007804466095, -0.002141460176, -0.0111365521, 0.01368706319, 0.00233457261, 0.01655124308, -0.2782102836, -0.02303841295, 0.002234097372, 0.01584899811, 0.02250744724, -0.002576682324, -0.3054474887, -0.2817180404, -2.985857003, 0.1547604111, -0.2624507116, -0.24948493, -3.000847553, 0.2254629612, -0.006056634615, -2.574751949, -0.3677979306, -0.00872760117, 0.01333960182, -3.86428792e-05, -0.0006964577897, -0.277073664, -0.01206040412, -0.009084143374, -0.01041442754, 0.0138919125, 0.002511853187, 0.01265550902, -0.288311415, -0.03336042298, 0.01463117526, 0.01622175126, 0.01924743474, -0.003536078104, -0.323673065, -0.2564383801, -2.968627243, 0.1976350009, -0.3475880726, -0.2501954361, -2.889092454, 0.3752726261, 0.2010438439, -2.327918128, -0.3579143727, -0.008188004443, 0.013317204, -0.0002167087678, -0.0007255725239, -0.2735838486, -0.007541084818, -0.008354560651, -0.001527489683, 0.01401923019, 0.002003528293, 0.01635095647, -0.2873956703, -0.01832685695, -0.009778327626, 0.01591826977, 0.01876642709, 0.009679861391, -0.3098844265, -0.2972868228, -3.060258518, 0.08974241141, -0.2722014974, 0.001726476394, -2.945106445, -0.03823666029, -0.2270137912, -2.521693759, -0.3524397033, -0.007996951386, 0.01331631189, -0.0001797054746, -0.001121326924, -0.2732231392, -0.006960128197, -0.00502591568, 0.0004913534184, 0.01358660888, 0.002483315542, 0.01482370909, -0.2758941133, -0.0305930929, 0.002133790591, 0.01613975594, 0.02656909888, 0.0001601150943, -0.3192757468, -0.2755258106, -3.041943355, 0.1377396797, -0.2999272363, -0.02895598934, -3.003260331, 0.2584431631, 0.006753159429, -2.406257766, -0.3614064032, -0.008323691431, 0.01329893124, -0.0001964631217, -0.0005924951544, -0.2702404738, -0.004983623112, -0.01043163006, 0.0007853285008, 0.01402430617, 0.002384307269, 0.01406497903, -0.2960537136, -0.02257235097, 0.01438742276, 0.01596158212, 0.01314855622, -0.005588607394, -0.3133450748, -0.2579050592, -3.07747383, 0.1619842443, -0.1882501829, -0.02202481688, -2.770434972, 0.2164803559, 0.1973931482, -2.472134111, -0.3637280321, -0.007807001295, 0.01318167897, -0.0003629538543, -0.0008902677909, -0.268113852, -0.0007226963445, -0.004422235643, 0.01057235643, 0.01375115638, 0.00229785923, 0.01853237878, -0.2806956299, -0.02795620879, -0.01037625048, 0.01593146366, 0.01615637856, -0.002359972828, -0.3128059763, -0.2874738207, -3.098808184, -0.006642729075, -0.1921096914, 0.2183082374, -2.954266182, 0.3648902354, -0.2202401322, -2.461719669, -0.3569789617, -0.007676454866, 0.01287582288, -0.0004938422173, -0.0009085134715, -0.2521003573, 0.005031006691, -0.001957496302, 0.01057969982, 0.01368049437, 0.00236900095, 0.0223499824, -0.2778833492, -0.03219239398, 0.00124729295, 0.01614402807, 0.02099707931, -0.0007979426115, -0.3208448401, -0.2900372934, -3.300224898, -0.1867763817, -0.4075647385, 0.2334792251, -2.986769814, 0.3597950025, -0.002393967334, -2.365079313, -0.3555996424, -0.009055047745, 0.01344970578, -0.0004028269089, -0.0007801636685, -0.2793177901, 0.0003003773248, -0.004360673292, 0.012509187, 0.01373559602, 0.002813242656, 0.0186390116, -0.2861549281, -0.04293294843, 0.01517057699, 0.01618495945, 0.02016748409, -0.01046971087, -0.3222295381, -0.2424189823, -2.982686636, -0.07798494321, -0.3082685593, 0.2004044748, -2.866607205, 0.5964769968, 0.1575895773, -2.420172916, -0.3627483746, 0.002946867727, 0.01353522332, -0.0003787761046, -0.0009036907238, -0.2836320998, -0.001745538761, -0.009010512891, -0.01337788917, 0.01360106737, 0.002523601994, 0.02348479221, -0.2736560026, -0.03612458606, -0.0120990837, 0.01603498447, 0.01897037923, -0.004302162433, -0.3172036838, -0.0672494132, -2.881740812, -0.1024134045, -0.2140237769, -0.2326410729, -3.066881396, 0.3820673378, -0.1655643421, -2.378368091, -0.3553213023, 0.00335077121, 0.01338057709, -0.0003674729062, -0.0008550950814, -0.2745298657, -4.970647096e-05, -0.00487645216, -0.01226954965, 0.01388883612, 0.002547468585, 0.02065928475, -0.2895329642, -0.03263392898, 0.0009425105369, 0.01601316932, 0.02216002912, -0.003870884216, -0.3153110879, -0.05198157891, -3.053110785, -0.05818190213, -0.3920544397, -0.2327271593, -2.824253719, 0.3316273869, 0.01148174641, -2.3833613, -0.36576133, 0.002456486699, 0.01338203453, -0.0003665105102, -0.0008276332188, -0.2766189347, -0.003158257325, -0.00623959147, -0.01126220455, 0.01388514495, 0.002435153542, 0.01851139511, -0.2923823058, -0.02716773562, 0.0140375024, 0.01621540017, 0.02236804003, -0.002597051692, -0.323621529, -0.005970877555, -2.973539755, -0.02270865191, -0.3276765518, -0.2463777271, -2.747103213, 0.2914715709, 0.186957964, -2.337341853, -0.3548231247, 0.001430166667, 0.01337878693, -0.0006536815811, -0.001034914491, -0.2745354216, 0.008675676681, 0.000414099509, -0.002433024993, 0.01359079194, 0.00254222654, 0.02779817111, -0.2736068154, -0.02707489128, -0.0106682397, 0.01599143508, 0.02618232594, -0.004824997974, -0.3131029944, -0.04092882652, -3.034989068, -0.2799380125, -0.4573947481, -0.001380952767, -3.010924355, 0.2477502324, -0.2049699079, -2.446533165, -0.3514063107, 0.002843690944, 0.01344859938, -0.000311377873, -0.0008484495795, -0.2796100558, -0.00105177389, -0.003413502837, -5.573414756e-05, 0.01385205722, 0.002580109241, 0.0207540953, -0.2864039811, -0.03339036609, 0.002007094994, 0.01587709233, 0.01771792579, -0.01089552358, -0.310611556, -0.0240268334, -2.96813075, -0.1148890056, -0.2615326758, -0.04082970973, -2.895499642, 0.4979064487, -0.01593995008, -2.462602169, -0.3523405498, 0.00199802798, 0.01330961494, -0.0004669285216, -0.001223369237, -0.2712800283, 0.004214413783, 0.002048260967, 0.001497507018, 0.01369960391, 0.002351172171, 0.02149001931, -0.2787504854, -0.02378251781, 0.01391295778, 0.01603912278, 0.02562773891, 0.001316302136, -0.3126777763, -0.00805225636, -3.092279263, -0.1303469235, -0.3639140971, -0.06201183829, -3.002007693, 0.09180064123, 0.1786303096, -2.475852894, -0.3593774469, 0.003555895695, 0.01318396096, -0.0003338505235, -0.0005695491961, -0.2676270378, -0.0001686090889, -0.01215468865, 0.009107745366, 0.01377854733, 0.002342105382, 0.02071129916, -0.2830418716, -0.02897655419, -0.01207683447, 0.01597293566, 0.01297219751, -0.00168932147, -0.3115161012, -0.07386793562, -3.079638237, -0.02425271704, -0.168679304, 0.2329236209, -2.94754842, 0.2979214695, -0.1821305602, -2.474685265, -0.35505666, 0.003400644117, 0.01323025702, -0.0006446901545, -0.000736350465, -0.2687828567, 0.007199847047, -0.009349654441, 0.009527520866, 0.01373182431, 0.00206522292, 0.02567344319, -0.2809480681, -0.01928352119, 0.001321089735, 0.01596743158, 0.01948105122, 0.006079992073, -0.314393686, -0.02947805449, -3.042509199, -0.2086157961, -0.2814743999, 0.2434948489, -2.952357108, 0.1127326166, -0.009932733256, -2.420276283, -0.3537338487, 0.002913740438, 0.01321104037, -0.0005830809604, -0.0007214273556, -0.2702500084, 0.0056026094, -0.008267688507, 0.0120449406, 0.01374963721, 0.00221642251, 0.02255698645, -0.2804430792, -0.02309167156, 0.01494107415, 0.0161098812, 0.01817963257, 0.000310509801, -0.3166140141, -0.02656630376, -3.035438037, -0.1471165575, -0.3066842328, 0.1901339245, -2.945573188, 0.2141156823, 0.1575483372, -2.464668509, -0.3642834334, 0.01247535959, 0.01314208157, -0.0006340079135, -0.00113873102, -0.2666971916, 0.003635018307, -0.002544411211, -0.01454951541, 0.01393086432, 0.0022439893, 0.02723135159, -0.2861194591, -0.02863215457, -0.01128108951, 0.01616443477, 0.02896284134, 0.0008080157748, -0.3196587636, 0.1908790969, -3.104232677, -0.1818602071, -0.4693732811, -0.2282517233, -2.923375239, 0.3057398077, -0.184799429, -2.386164327, -0.3644937229, 0.01431106593, 0.01349332385, -9.517510272e-05, -0.0007650802377, -0.2762164482, -0.01415865368, -0.004260672393, -0.01264573534, 0.01381343353, 0.002354170742, 0.01401189023, -0.2841313707, -0.02621318718, 0.001035910529, 0.01621803574, 0.01863791665, 0.003930959749, -0.3303331651, 0.1747671064, -3.037995145, 0.2748265701, -0.3884124312, -0.2524615252, -2.90462411, 0.159406315, -0.004995010033, -2.17667117, -0.364503058, 0.01411805928, 0.01342488709, -0.0002097200427, -0.0008770423092, -0.2769531628, -0.002534737696, -0.006885120415, -0.01069355759, 0.01391503771, 0.002540821753, 0.01259500844, -0.2901833001, -0.03519991875, 0.01457891709, 0.01640825976, 0.0276081624, -0.003806697781, -0.3321743102, 0.2004502933, -3.033340508, 0.161055699, -0.4576727181, -0.2771860449, -2.888163908, 0.3385367515, 0.1754285802, -2.25800797, -0.3620955228, 0.01261507596, 0.0134132516, -0.0003369609271, -0.0007722280931, -0.282955171, -0.003856777719, -0.01083651705, -0.002907691909, 0.01383005854, 0.00222077822, 0.01639854175, -0.2879331277, -0.02499649746, -0.01213871376, 0.01619727097, 0.01582014317, 0.001928975154, -0.3263671155, 0.2036347568, -2.841376034, 0.155559506, -0.1643138077, 0.004626916507, -2.870415874, 0.2319107089, -0.1772091811, -2.249295532, -0.3542442042, 0.01317103755, 0.0134937766, -0.0004070548062, -0.0008082123858, -0.2802280975, -0.003247773857, -0.009592605722, -0.00140795262, 0.01363195344, 0.002376807878, 0.02159567094, -0.278648121, -0.02371157322, 0.0006793110005, 0.01595161359, 0.02035378775, -0.001067458102, -0.3063724697, 0.2103899846, -2.959292336, -0.02404242783, -0.2477506299, -0.02875223006, -2.948157871, 0.2048445271, 0.001740886286, -2.570337449, -0.3571173958, 0.0132233933, 0.01336413523, -0.0003252033204, -0.0007567745086, -0.2758227409, -0.001545507567, -0.006869208529, -0.000359042549, 0.0137510687, 0.002010024439, 0.01551372023, -0.2835988761, -0.01373440139, 0.0128855165, 0.01598620666, 0.02152035608, 0.00405105664, -0.3136783786, 0.2172274125, -2.994660099, 0.03903634681, -0.334104822, -0.03975725202, -2.904818994, 0.03105168579, 0.2013155427, -2.46829352, -0.3634682017, 0.01388538682, 0.01353078321, -0.000398100822, -0.0006931920154, -0.2795720244, -0.00561698407, -0.01140385791, 0.008125224593, 0.01382718965, 0.002271749201, 0.02047127296, -0.2882777527, -0.02181576218, -0.0108810166, 0.01638765439, 0.0150532365, -0.0008809094834, -0.3329108748, 0.1842306738, -3.029225593, 0.08879147543, -0.2240460574, 0.2251433575, -2.853661167, 0.2302933317, -0.2202935012, -2.305485085, -0.3548589277, 0.01355925982, 0.01329039819, -0.00052633344, -0.001165768504, -0.27433702, 0.001257913318, 0.002204766093, 0.009392104058, 0.01354245146, 0.002249274101, 0.02224688768, -0.2704145783, -0.02098156961, -0.0003071322915, 0.0162905695, 0.0279183801, 0.003143254254, -0.3275139602, 0.2074979378, -2.989922076, -0.1109264208, -0.421087535, 0.216222828, -3.077033554, 0.1162454158, 0.02399669093, -2.318671574, -0.3659728311, 0.01363330485, 0.01354029237, -0.0004155008672, -0.0004699867419, -0.2841278199, -0.003816380129, -0.01526254839, 0.009998095692, 0.01364192326, 0.002544744116, 0.02243279342, -0.2733371983, -0.03090219921, 0.01331521497, 0.01606630319, 0.01260696741, -0.002084209214, -0.3180318277, 0.2194364501, -2.902704969, 0.01963373894, -0.1288176795, 0.2358334667, -3.056888839, 0.2543009508, 0.1721559575, -2.380628783;

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
