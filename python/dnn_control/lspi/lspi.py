# Least Squares Policy Iteration
from numpy import zeros, array
from pendulum import PENDULUM_PHI_SIZE, pendulum_pi, pendulum_phi, pendulum_prepare_samples
from spacecraft import SPACECRAFT_PHI_SIZE, spacecraft_pi, spacecraft_phi, spacecraft_prepare_samples, spacecraft_test

# NUM_SAMPLES: the number of samples we have available
NUM_SAMPLES = 50

# NUM_STEPS: the number of steps taken from an intial sample point
NUM_STEPS = 200

# STATE_ACTION_DIMENSIONS: Number of dimensions in the transformed state action space
STATE_ACTION_DIMENSIONS = SPACECRAFT_PHI_SIZE

# PHI: Function which transforms state action pairs into the STATE_ACTION_DIMENSIONS state action space
PHI = spacecraft_phi

# Policy of the system
POLICY = spacecraft_pi

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
    from numpy.linalg import inv, matrix_rank, pinv

    A = matrix(zeros([STATE_ACTION_DIMENSIONS, STATE_ACTION_DIMENSIONS]))
    b = zeros([STATE_ACTION_DIMENSIONS, 1])

    for (s, a, r, s_prime) in D:
        phi_sa = phi(s, a)
        a_prime = pi(s_prime, w)
        phi_sa_prime = phi(s_prime, a_prime)
        A = A + dot(phi_sa, (phi_sa - GAMMA * phi_sa_prime).T)
        b = b + phi_sa * r

    rank = matrix_rank(A, 1e-10)

    if STATE_ACTION_DIMENSIONS == rank:
        print("Matrix A has full rank {0}.".format(rank))
        return inv(A) * b
    else:
        print("Matrix A has rank {0}. Number of state action dimensions would be {1}".format(rank, STATE_ACTION_DIMENSIONS))
        return pinv(A) * b


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

D, asteroid, simulator = spacecraft_prepare_samples(NUM_SAMPLES, NUM_STEPS)
print("Generated {0} samples. Training policy ... ".format(len(D)))

w = lspi(D, GAMMA, EPSILON, W)

print("done. Testing policy ... ")
#positions, heights = spacecraft_test(w, semi_axis, simulator, control_frequency,
#                                     int(control_frequency * 60.0 * 60.0 * 2.0))
#print("done. Writing result file ...")
#from filewriter import FileWriter
#writer = FileWriter()
#writer.create_visualization_file("../../../results/states.txt", control_frequency, semi_axis, positions, heights)
#print("done.")