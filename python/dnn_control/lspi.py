# Least Squares Policy Iteration
from numpy import zeros, array


def ncr(n, r):
    import operator as op
    r = min(r, n-r)
    if r == 0: return 1
    numer = reduce(op.mul, xrange(n, n-r, -1))
    denom = reduce(op.mul, xrange(1, r+1))
    return numer//denom

# STATE_DIMENSION: dimension of the state
STATE_DIMENSION = 6

# NUM_SAMPLES: the number of samples we have available
NUM_SAMPLES = 50000

# ACTIONS: discrete actions
ACTIONS = {0: [20.0, 0.0, 0.0],
           1: [-20.0, 0.0, 0.0],
           2: [0.0, 20.0, 0.0],
           3: [0.0, -20.0, 0.0],
           4: [0.0, 0.0, 20.0],
           5: [0.0, 0.0, -20.0],
           6: [0.0, 0.0, 0.0]}

# DEGREE_POLYNOMIAL: highest degree polynomial
DEGREE_POLYNOMIAL = 2

STATE_ACTION_DIMENSIONS = len(ACTIONS) * (ncr(DEGREE_POLYNOMIAL + STATE_DIMENSION - 1, STATE_DIMENSION - 1) + 1)

# W: parameters of parametric approximation of Q
W = zeros([STATE_ACTION_DIMENSIONS, 1])

# GAMMA: learning parameter
GAMMA = 0.9

# EXPLORE: exploration exploitation tradeoff
EXPLORE = 0.5

# EPSILON:
EPSILON = 1e-10


# Our finite dimensional state-action space
def phi(s, a):
    from numpy import zeros
    result = zeros([STATE_ACTION_DIMENSIONS, 1])
    base = a * (ncr(DEGREE_POLYNOMIAL + STATE_DIMENSION - 1, STATE_DIMENSION - 1) + 1)

    result[base] = 1.0
    base += 1
    num_polys = 0
    for pow_0 in range(DEGREE_POLYNOMIAL + 1):
        for pow_1 in range(DEGREE_POLYNOMIAL + 1 - pow_0):
            for pow_2 in range(DEGREE_POLYNOMIAL + 1 - pow_0 - pow_1):
                for pow_3 in range(DEGREE_POLYNOMIAL + 1 - pow_0 - pow_1 - pow_2):
                    for pow_4 in range(DEGREE_POLYNOMIAL + 1 - pow_0 - pow_1 - pow_2 - pow_3):
                        pow_5 = DEGREE_POLYNOMIAL + 1 - pow_0 - pow_1 - pow_2 - pow_3 - pow_4
                        result[base] = s[0] ** pow_0 * s[1] ** pow_1 * s[2] ** pow_2 \
                                       * s[3] ** pow_3 * s[4] ** pow_4 * s[5] ** pow_5
                        base += 1
                        num_polys += 1
    return result


# policy of system: to be optimized
def pi(s, w):
    from numpy import random
    from numpy import empty, copy, dot
    from sys import float_info

    if random.rand() < EXPLORE:
        a = random.choice(len(ACTIONS))
        return a
    else:
        best_a = random.choice(len(ACTIONS))
        best_q = 0.0
        for a in ACTIONS:
            val_phi = phi(s, a).T
            q = dot(val_phi, w)[0, 0]
            if q > best_q:
                best_q = q
                best_a = a

        return best_a


def lstdq(D, gamma, w):
    from numpy import matrix, zeros, dot
    from numpy.linalg import inv, matrix_rank

    A = matrix(zeros([STATE_ACTION_DIMENSIONS, STATE_ACTION_DIMENSIONS]))
    b = zeros([STATE_ACTION_DIMENSIONS, 1])

    for (s, a, r, s_prime) in D:
        phi_sa = phi(s, a)
        phi_sa_prime = phi_sa - gamma * phi(s_prime, pi(s, w))
        A = A + dot(phi_sa, phi_sa_prime.T)
        b = b + phi_sa * r

    print("Rank of A: {0}".format(matrix_rank(A)))
    return inv(A) * b


def lspi(D, gamma, epsilon, w):
    from numpy.linalg import norm
    from numpy import copy

    iteration = 0
    w_prime = w
    val_norm = 1.0
    while val_norm > epsilon:
        print("iteration {0}. Norm: {1}".format(iteration, val_norm))
        w = w_prime
        w_prime = lstdq(D, gamma, w)
        val_norm = norm(w_prime - w)
        iteration += 1

    return w

def prepare_samples(num_samples):
    from math import sqrt
    from numpy import random, array
    from utility import sample_sign
    from utility import sample_point_outside_ellipsoid
    from boost_simulator import boost_simulator
    Simulator = boost_simulator.BoostSimulator

    samples = []

    semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]
    density = random.uniform(1500.0, 3000.0)
    angular_velocity = [sample_sign() * random.uniform(0.0002, 0.0008),
                        0.0,
                        sample_sign() * random.uniform(0.0002, 0.0008)]
    time_bias = 0.0

    spacecraft_specific_impulse = 200.0

    control_frequency = 0.1

    target_position = sample_point_outside_ellipsoid(semi_axis, 4.0)

    sensor_noise = 0.05
    perturbation_noise = 1e-7
    control_noise = 0.05

    simulator = Simulator(semi_axis, density, angular_velocity, time_bias,
                          target_position, control_frequency, sensor_noise, perturbation_noise, control_noise)

    simulator.init_spacecraft_specific_impulse(spacecraft_specific_impulse)

    for i in xrange(num_samples):
        position = sample_point_outside_ellipsoid(semi_axis, 4.0)
        velocity = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
        mass = random.uniform(500.0, 1500.0)
        a = random.choice(len(ACTIONS))
        thrust = ACTIONS[a]
        t = random.uniform(0.0, 24.0 * 60.0 * 60.0)
        state = position + velocity + [mass]
        next_state = simulator.next_state(state, thrust, t)
        next_pos = next_state[0:3]

        r = 1.0 / (1.0 + sqrt(sum([(p-np) ** 2 for p, np in zip(position, next_pos)])))

        sample = (array(state), a, r, array(next_state))
        samples.append(sample)

    return samples


D = prepare_samples(NUM_SAMPLES)
result = lspi(D, GAMMA, EPSILON, W)
print("")
print(result)