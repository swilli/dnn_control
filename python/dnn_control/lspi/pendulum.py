# Pendulum LSPI

PENDULUM_PHI_SIZE = 30

PENDULUM_l = 0.5
PENDULUM_M = 8.0
PENDULUM_m = 2.0
PENDULUM_ACTIONS = {
    0: -50.0,
    1: 0.0,
    2: 50.0
}



def d_state_dt(time, state, action):
    from constants import EARTH_ACCELERATION
    from math import sin, cos

    a = 1.0 / (PENDULUM_m + PENDULUM_M)

    result = [0.0, 0.0]
    result[0] = state[1]
    result[1] = (EARTH_ACCELERATION * sin(state[0]) - a * PENDULUM_m * PENDULUM_l * state[1] * state[1]
                 * sin(2.0 * state[0]) / 2.0 - a * cos(state[0]) * action) \
                / (4.0 / 3.0 * PENDULUM_l - a * PENDULUM_m * PENDULUM_l * cos(state[0]) ** 2)

    return result


def pendulum_next_state(state, action):
    from scipy.integrate import ode
    integrator = ode(d_state_dt)
    integrator.set_integrator("lsoda")
    integrator.set_initial_value(state, 0.0)
    integrator.set_f_params(PENDULUM_ACTIONS[action])
    integrator.integrate(0.1)
    return integrator.y[:]


def pendulum_phi(s, a):
    from numpy import zeros
    from math import acos, exp
    from constants import PI

    result = zeros([30, 1])
    base = a * 10

    result[base] = 1.0
    base += 1
    for x in [-PI / 4.0, 0.0, PI / 4.0]:
        for y in [-1.0, 0.0, 1.0]:
            dist = (s[0] - x) * (s[0] - x) + (s[1] - y) * (s[1] - y)
            result[base] = exp(-dist * 0.5)
            base += 1

    return result


def pendulum_initialize_state():
    from numpy import random
    return [random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)]


def pendulum_pi(s, w):
    from numpy import random
    from numpy import dot
    from sys import float_info

    best_a = [random.choice([0, 1, 2])]
    best_q = -float_info.max
    for a in PENDULUM_ACTIONS:
        val_phi = pendulum_phi(s, a).T
        q = dot(val_phi, w)[0, 0]
        if q > best_q:
            best_q = q
            best_a = [a]
        elif q == best_q:
            best_a.append(a)

    return random.choice(best_a)


def pendulum_prepare_samples(num_samples, num_steps):
    from numpy import random, array
    from constants import PI

    samples = []

    for i in xrange(num_samples):
        state = pendulum_initialize_state()
        for j in xrange(num_steps):
            a = random.choice([0, 1, 2])
            next_state = pendulum_next_state(state, a)
            # r = abs(state[0]) - abs(next_state[0])
            end_sim = abs(next_state[0]) > PI / 2.0
            if end_sim:
                r = -1.0
                sample = (array(state), a, r, array(next_state))
                samples.append(sample)
                break
            else:
                r = 0.0
                sample = (array(state), a, r, array(next_state))
                samples.append(sample)
                state = next_state

    return samples