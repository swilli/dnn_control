# Least Squares Policy Iteration
from numpy import zeros, array
from pendulum import PENDULUM_PHI_SIZE, pendulum_pi, pendulum_phi, prepare_samples_pendulum


# NUM_SAMPLES: the number of samples we have available
NUM_SAMPLES = 1000

# NUM_STEPS: the number of steps taken from an intial sample point
NUM_STEPS = 50

# STATE_ACTION_DIMENSIONS: Number of dimensions in the transformed state action space
STATE_ACTION_DIMENSIONS = PENDULUM_PHI_SIZE

# PHI: Function which transforms state action pairs into the STATE_ACTION_DIMENSIONS state action space
PHI = pendulum_phi

# Policy of the system
POLICY = pendulum_pi

# W: parameters of parametric approximation of Q
W = zeros([STATE_ACTION_DIMENSIONS, 1])

# GAMMA: discount parameter
GAMMA = 0.9

# EPSILON: Defines when the system has converged
EPSILON = 1e-20


# policy of system defined by weights in w
def pi(s, w):
    return POLICY(s, w)


# state action kernel
def phi(s, a):
    return PHI(s, a)


def lstdq(D, gamma, w):
    from numpy import matrix, zeros, dot
    from numpy.linalg import inv, matrix_rank

    A = matrix(zeros([STATE_ACTION_DIMENSIONS, STATE_ACTION_DIMENSIONS]))
    b = zeros([STATE_ACTION_DIMENSIONS, 1])

    for (s, a, r, s_prime) in D:
        phi_sa = phi(s, a)
        a_prime = pi(s_prime, w)
        phi_sa_prime = phi(s_prime, a_prime)
        A = A + dot(phi_sa, (phi_sa - GAMMA * phi_sa_prime).T)
        b = b + phi_sa * r

    print("STATE_ACTION_DIMENSIONS: {0}, Rank of A: {1}".format(STATE_ACTION_DIMENSIONS, matrix_rank(A)))
    return inv(A) * b


def lspi(D, gamma, epsilon, w):
    from numpy.linalg import norm

    iteration = 0
    w_prime = w
    val_norm = 1.0
    while val_norm > epsilon:
        print("iteration {0}. Norm: {1}".format(iteration, val_norm))
        w = w_prime
        w_prime = lstdq(D, gamma, w)
        val_norm = norm(w_prime - w)
        iteration += 1

    print("iteration {0}. Norm: {1}".format(iteration, val_norm))
    return w


def prepare_samples_spacecraft(num_samples):
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

        err_state = sqrt(sum([(p-np) ** 2 for p, np in zip(target_position, position)]))
        err_next_state = sqrt(sum([(p-np) ** 2 for p, np in zip(target_position, next_pos)]))

        r = err_state - err_next_state

        sample = (array(state), a, r, array(next_state))
        samples.append(sample)

    return samples


D = prepare_samples_pendulum(NUM_SAMPLES, NUM_STEPS)
print("Generated {0} samples".format(len(D)))
result = lspi(D, GAMMA, EPSILON, W)
print("")
print(result)