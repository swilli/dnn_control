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

    const double dt = simulator.InteractionInterval();
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
    const double dt = simulator.InteractionInterval();

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
    SampleFactory sample_factory(random_seed + 1);
    const Vector3D target_position = sample_factory.SamplePointOutSideEllipsoid(simulator.AsteroidOfSystem().SemiAxis(), 1.1, 4.0);


    eigen::VectorXd weights(kSpacecraftPhiSize);
    weights << -0.3840859217, -0.012680189, 0.01441057939, -0.001238134891, 0.0007195330853, -0.3044666094, -0.01299022753, -0.01332679646, -0.009631242011, 0.01589671329, 0.001651883682, 0.03395086455, -0.3310103811, -0.01341624349, -0.01114179795, 0.01516679454, 0.01011024613, -0.0252183606, -0.3108090233, -0.2214825463, -2.301648878, -0.1381585993, 0.01900650299, -0.1521243462, -2.244857992, 0.3677436633, -0.1673942249, -2.420089397, -0.3752864677, -0.01222652403, 0.01422188965, -0.001195188514, 0.0007651909185, -0.2942149697, -0.007265639533, -0.008139733824, -0.009702210986, 0.01586861639, 0.001512184891, 0.02509956759, -0.331789352, -0.01241723371, -5.658531544e-05, 0.01513624728, 0.007648577247, -0.02047639046, -0.3075058552, -0.2249499352, -2.455802836, -0.09858644634, -0.02339999175, -0.1599841295, -2.200458475, 0.4393327868, 0.06160443534, -2.517398726, -0.3727557466, -0.0107810176, 0.01427502209, -0.0007579358553, 0.0006825968545, -0.2927137269, -0.01963659858, -0.005251293487, -0.008903550794, 0.01548179915, 0.001344160938, 0.01451210771, -0.3128125289, -0.002501736138, 0.01210463466, 0.01479399005, 0.005352865615, -0.01902853263, -0.291860965, -0.2440690229, -2.550895437, 0.1995878395, 0.09824337698, -0.197822729, -2.449138895, 0.1969101172, 0.2413657433, -2.680372851, -0.3697391543, -0.01139782119, 0.01450811152, -0.001016100389, 0.0007029860697, -0.3043381058, -0.01230087466, -0.008327656281, 0.002830827936, 0.0155459589, 0.001266495441, 0.02705668756, -0.3136591515, 0.0008653036107, -0.01081749367, 0.01482070516, 0.006934772071, -0.01688334594, -0.2982967082, -0.2398387463, -2.390442681, -0.03567973073, 0.01847423298, 0.03601549168, -2.463393193, 0.1647731955, -0.1739997904, -2.533424942, -0.3766066219, -0.01122221535, 0.01436249009, -0.000638631931, 0.0005335336243, -0.2975742701, -0.02766717563, -0.001855359725, 0.004221402134, 0.01590619516, 0.001268759443, 0.01865883617, -0.3301880765, -0.000165424208, -0.0001041722776, 0.01491381887, 0.00878211433, -0.01460384352, -0.3044374948, -0.2171941612, -2.490175141, 0.2649425138, -0.09985935357, 0.002978743135, -2.245777063, 0.1407790151, 0.06627400814, -2.382161784, -0.3762257859, -0.01090852111, 0.01408402498, -0.001293234738, 0.0009265422009, -0.2836058628, -0.01112552643, -0.01242158083, 0.003528628061, 0.01568910818, 0.0009556830795, 0.03058510074, -0.3234372079, 0.003275803505, 0.01238478844, 0.01486759887, 0.001020438549, -0.008444571861, -0.2993262415, -0.2446095471, -2.628915818, -0.02728032925, 0.1437358419, -0.0004707366889, -2.286740218, 0.006011725828, 0.2602380866, -2.547470128, -0.3761989668, -0.01071216868, 0.01416834695, -0.0008231172041, 0.0008055285512, -0.2871789159, -0.02019105914, -0.01219808173, 0.01510758753, 0.01558972603, 0.001337597198, 0.01939990197, -0.3152968484, -0.005217318375, -0.009975053668, 0.01482187941, 0.004319325394, -0.0185795202, -0.2959371298, -0.2381928653, -2.634305636, 0.1462559083, 0.2174941412, 0.2373232321, -2.42229561, 0.2908952855, -0.174936766, -2.58743703, -0.3787974785, -0.009442268212, 0.01424118289, -0.0009234692552, 0.0006753051088, -0.2926463745, -0.02207573989, -0.004325753262, 0.0152464879, 0.01582740002, 0.001743290395, 0.02230928393, -0.3270880513, -0.01499287122, 0.001837639316, 0.01494607622, 0.01257287813, -0.02709669053, -0.297867767, -0.2477167545, -2.500779758, 0.2009729714, -0.1200208925, 0.2303927022, -2.284455695, 0.4101835992, 0.02208540072, -2.645071393, -0.3809244763, -0.00942735913, 0.01411303474, -0.0008470013186, 0.0008871614909, -0.2867511616, -0.01755596413, -0.01662969067, 0.01743792664, 0.0157057041, 0.001093797974, 0.01916101808, -0.3231358005, 0.001525533753, 0.01327452445, 0.01498888879, 0.001329391472, -0.009203822306, -0.3038958642, -0.2484643062, -2.614724088, 0.06056485275, 0.2745990073, 0.1853892148, -2.291659807, -0.0184665486, 0.2391681184, -2.461704492, -0.3728982953, -0.001459659481, 0.01438917357, -0.000925336782, 0.001133114521, -0.2971102153, -0.02180007269, -0.01966534934, -0.01036897395, 0.01566585544, 0.001207576176, 0.02194358725, -0.3181727853, 0.00188467102, -0.01192411411, 0.01500863699, -0.007918985808, -0.01546373352, -0.3040330159, -0.0112346574, -2.486609693, 0.1832871424, 0.4493778743, -0.1534969021, -2.379697617, 0.1043093483, -0.144718619, -2.503716247, -0.3699942441, 0.001723571321, 0.01431312249, -0.0008360787489, 0.0006945540073, -0.2963426284, -0.01677993515, -0.006174466535, -0.0103618584, 0.01562944111, 0.001189310661, 0.01663205801, -0.3160127996, 0.00481566518, 0.0009635132416, 0.01504074851, 0.006190003826, -0.01067428951, -0.308887877, -0.07861119363, -2.470788799, 0.1512781324, 0.0898272242, -0.1732656166, -2.432604496, -0.05035569223, 0.005108697085, -2.432922678, -0.3724468472, -0.0001157624318, 0.01424305818, -0.0007345484142, 0.0008044469522, -0.2918597745, -0.02230583307, -0.007828880057, -0.01072328389, 0.01568106078, 0.001332738736, 0.01480049083, -0.3211492805, -0.008057547566, 0.01207211002, 0.01506553913, 0.001538303572, -0.01608762654, -0.3083779609, -0.04737802021, -2.588678867, 0.2187894751, 0.1036047913, -0.1689384784, -2.338271822, 0.2274895548, 0.2356823336, -2.437316522, -0.3738351824, 0.0008986271201, 0.01428102842, -0.0007539019558, 0.0006712437637, -0.2931081091, -0.02223879834, -0.006368721951, 0.002702094702, 0.01589204209, 0.001573959614, 0.01650430706, -0.3304503334, -0.01013455124, -0.01096526145, 0.01504999324, 0.006322506988, -0.02245724777, -0.306790873, -0.06573805969, -2.50119714, 0.2139371373, 0.04236881113, 0.016593748, -2.263261028, 0.3389643671, -0.1880945956, -2.467869083, -0.3645879166, -0.0003815377671, 0.01429540877, -0.0009032631706, 0.0009439701976, -0.2960483164, -0.01746799843, -0.01305582313, 0.003170354011, 0.0155918534, 0.001178454209, 0.01426056316, -0.3137860046, -0.0009776328686, 0.0007274785475, 0.01497903822, 0.001824115623, -0.008369992422, -0.3049176716, -0.02669706485, -2.460013475, 0.2282027012, 0.129228169, 0.008040971524, -2.505363119, -0.02042776447, 0.02427848217, -2.416358713, -0.3725217864, 0.000724453042, 0.01424088617, -0.0008976778972, 0.0007384061246, -0.2921954402, -0.01708051773, -0.00489509492, 0.002368821666, 0.01575112719, 0.001420201898, 0.02430016927, -0.3262100242, -0.006569374269, 0.01261524206, 0.0147595266, 0.008959710696, -0.01766821786, -0.292079143, -0.03286235576, -2.502522597, -0.02313675656, -0.1443099857, 0.0255561551, -2.250910187, 0.1510956726, 0.2327233076, -2.676732879, -0.373444643, 0.0005008666193, 0.01421820848, -0.0007614667899, 0.0008373737172, -0.2870291235, -0.02346698696, -0.009530418662, 0.01534467331, 0.0157450985, 0.001487973692, 0.01496334315, -0.3262811483, -0.005116165502, -0.01046250997, 0.01495031094, 0.004064967849, -0.02241193462, -0.3020017992, -0.02122801579, -2.618784794, 0.2994966054, 0.1429129124, 0.2077491081, -2.271201712, 0.2612521265, -0.1851229998, -2.508749517, -0.3684392284, 0.001803375924, 0.01418941618, -0.00128655942, 0.0009366500426, -0.290142169, -0.006990997693, -0.01420028901, 0.01492141573, 0.01558639277, 0.001021242344, 0.02645418074, -0.313835226, 0.00656578206, 0.001342284753, 0.0150392624, 0.003063279752, -0.006012273537, -0.3100349147, -0.06441335057, -2.572009556, -0.01816004923, 0.1528666635, 0.1968245435, -2.495902452, -0.183888225, 0.02968996384, -2.340163981, -0.3723851626, 0.001356026946, 0.01408894736, -0.001207623475, 0.0009660836718, -0.2843820127, -0.00548467768, -0.01253514452, 0.01400348625, 0.01583165082, 0.001290100285, 0.02799077156, -0.3320547206, -0.006777279423, 0.01479455512, 0.0148592885, -0.0001038362602, -0.01624790429, -0.2983603915, -0.04513355076, -2.605723519, -0.2267643399, 0.1792885424, 0.2222940133, -2.197865996, 0.1117017925, 0.188756981, -2.594258496, -0.3737252665, 0.01144542962, 0.01424909616, -0.0009067104412, 0.001101278041, -0.2958701263, -0.01391177107, -0.01700544988, -0.01160976491, 0.01565839683, 0.001141685923, 0.02347253635, -0.3223220135, 0.0009116798272, -0.01278237856, 0.01507446285, 0.0006449524758, -0.01508889558, -0.3036386773, 0.1605861595, -2.468455512, -0.08524970643, 0.2212447214, -0.156129406, -2.32554453, 0.1236380284, -0.1553040077, -2.544208188, -0.36856896, 0.01198316069, 0.01417768477, -0.001006187675, 0.0005480184791, -0.2916697314, -0.01216677283, -0.003029216901, -0.01090997982, 0.01550140345, 0.001637659747, 0.02525155408, -0.3138964305, -0.01407557818, -0.0003230178186, 0.01517473411, 0.01075638043, -0.02981099443, -0.3115170901, 0.1336485555, -2.498331407, -0.0776167844, -0.09047292114, -0.1686994961, -2.406462378, 0.534174332, 0.0470153226, -2.459383952, -0.3737187889, 0.01275016352, 0.0142853471, -0.001320624407, 0.0008320482581, -0.2930046545, -0.007906594799, -0.0141387356, -0.01162434945, 0.01584161557, 0.001174634722, 0.03515326473, -0.3283774411, -0.0004568664967, 0.01218909294, 0.01491694143, 0.004982103752, -0.01329654697, -0.2967204117, 0.1247215641, -2.557840626, -0.1891666939, 0.1235503697, -0.1736269893, -2.268783403, 0.1440603046, 0.2248152418, -2.650947274, -0.3675521635, 0.01180578695, 0.01416015743, -0.001003816468, 0.0004272699386, -0.2870122405, -0.01538368813, -0.001535333386, 0.000721391511, 0.01554642614, 0.001446651688, 0.02026755218, -0.3164826448, -0.006975878055, -0.01135527977, 0.01495535921, 0.01150998723, -0.01933185562, -0.298268242, 0.161895113, -2.584552325, 0.08084737323, -0.01307837055, 0.03950677924, -2.4124706, 0.2241329644, -0.1741777471, -2.617377514, -0.368672476, 0.01157713961, 0.01450003115, -0.0007580417167, 0.0002903585611, -0.3020112703, -0.02058714548, 0.001584435629, 0.0009594087762, 0.0158162635, 0.001115491286, 0.01777784045, -0.329945546, -0.002324316141, 0.0001925456292, 0.01509604822, 0.01750155922, -0.007645569755, -0.3087791, 0.1787463761, -2.449487461, 0.1435980366, -0.1871493503, 0.04034640261, -2.288197227, 0.06402252173, 0.03307859083, -2.454470142, -0.3746208683, 0.01182542321, 0.01420217582, -0.000764729301, 0.0002715866135, -0.2878009104, -0.02308949464, 0.005270864001, 0.001875835987, 0.01551632673, 0.001264452048, 0.0153013259, -0.3106635977, -0.006170850425, 0.012127065, 0.01492881218, 0.0192753598, -0.01249825566, -0.2992141475, 0.1744076795, -2.589646585, 0.2641956862, -0.3108882206, 0.01132397853, -2.474509126, 0.1313172334, 0.2359698037, -2.55949373, -0.3709133154, 0.01225613618, 0.01415160108, -0.0009544976924, 0.0006077180729, -0.2873678335, -0.01714897473, -0.005686443156, 0.01343814621, 0.01588349123, 0.001387878434, 0.02407096038, -0.3306053869, -0.0039549546, -0.01099727305, 0.01481617261, 0.008350071367, -0.01772219961, -0.2942781305, 0.164757041, -2.605900852, 0.1000648179, -0.03990230232, 0.2133946919, -2.265982297, 0.2258011603, -0.1781401637, -2.665721247, -0.3774313167, 0.01240805033, 0.01426041899, -0.001023790743, 0.0009184147813, -0.2941268225, -0.01665071336, -0.01140443905, 0.01326440544, 0.01583745399, 0.001165246041, 0.02152163397, -0.3265150804, -0.0009031465572, -5.603765074e-05, 0.01502691742, 0.001398595895, -0.009622451187, -0.3079707694, 0.181534259, -2.4937334, 0.1587827629, 0.1332796329, 0.2268888338, -2.326559546, 0.04771556395, 0.06772475509, -2.413767282, -0.3802103921, 0.01318815942, 0.01429262033, -0.001130562769, 0.001145920914, -0.2974756553, -0.01405574641, -0.02419598316, 0.01473644296, 0.0158498542, 0.001562368833, 0.0267202879, -0.3275381496, -0.008386361172, 0.01331230704, 0.01502905995, -0.004528686751, -0.02607884897, -0.3087555139, 0.1673942951, -2.436441974, 0.08641498347, 0.4448051345, 0.1851272511, -2.301402045, 0.3602409498, 0.2131460492, -2.43385555;

    const unsigned int test_seed = sample_factory.SampleRandomInteger();
    std::cout << "testing policy ... ";
    const boost::tuple<std::vector<Vector3D>, std::vector<Vector3D> > result = TestPolicy(test_seed, weights, simulator, target_position, test_time / simulator.InteractionInterval());
    std::cout << "done." << std::endl << "writing result to file ... ";

    const std::vector<Vector3D> &positions = boost::get<0>(result);
    const std::vector<Vector3D> &heights = boost::get<1>(result);

    FileWriter writer;
    writer.CreateVisualizationFile(PATH_TO_LSPI_VISUALIZATION_FILE, simulator.InteractionInterval(), simulator.AsteroidOfSystem(), positions, heights);
    std::cout << "done." << std::endl;
}

void TrainLeastSquaresPolicyController(const unsigned int &random_seed) {
    std::cout << "LSPI seed: " << random_seed << std::endl;

    Init();

    const unsigned int num_samples = 1000;
    const unsigned int num_steps = 50;

    const double gamma = 0.9;
    const double epsilon = 1e-5;

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
