# Spacecraft LSPI


def ncr(n, r):
    import operator as op
    r = min(r, n-r)
    if r == 0: return 1
    numer = reduce(op.mul, xrange(n, n-r, -1))
    denom = reduce(op.mul, xrange(1, r+1))
    return numer//denom

SPACECRAFT_STATE_DIMENSION = 6

SPACECRAFT_ACTIONS = {
    0: [-50.0, 0.0, 0.0],
    1: [50.0, 0.0, 0.0],
    2: [0.0, -50.0, 0.0],
    3: [0.0, 50.0, 0.0],
    4: [0.0, 0.0, -50.0],
    5: [0.0, 0.0, 50.0],
    6: [0.0, 0.0, 0.0]
}

SPACECRAFT_POLYNOMIAL_DIMENSIONS = 58

SPACECRAFT_PHI_SIZE = len(SPACECRAFT_ACTIONS) * SPACECRAFT_POLYNOMIAL_DIMENSIONS


def spacecraft_phi(s, a):
    from numpy import zeros

    result = zeros([SPACECRAFT_PHI_SIZE, 1])
    base = a * SPACECRAFT_POLYNOMIAL_DIMENSIONS

    result[base] = 1.0
    base += 1
    for i in range(SPACECRAFT_STATE_DIMENSION):
        result[base] = s[i]
        base += 1
        result[base] = s[i] * s[i]
        base += 1

    for i in range(SPACECRAFT_STATE_DIMENSION):
        for j in range(i + 1, SPACECRAFT_STATE_DIMENSION):
            result[base] = s[i] * s[j]
            base += 1
            result[base] = s[i] * s[i] * s[j]
            base += 1
            result[base] = s[i] * s[j] * s[j]
            base += 1

    return result


def spacecraft_initialize_state(semi_axis):
    from numpy import random
    from utility import sample_point_outside_ellipsoid

    position = sample_point_outside_ellipsoid(semi_axis, 4.0)
    velocity = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
    mass = random.uniform(500.0, 1500.0)
    return position + velocity + [mass]


def spacecraft_pi(s, w):
    from numpy import random
    from numpy import dot
    from sys import float_info

    best_a = [random.choice([0, 1, 2])]
    best_q = -float_info.max
    for a in SPACECRAFT_ACTIONS:
        val_phi = spacecraft_phi(s, a).T
        q = dot(val_phi, w)[0, 0]
        if q > best_q:
            best_q = q
            best_a = [a]
        elif q == best_q:
            best_a.append(a)

    return random.choice(best_a)


def spacecraft_test(w, semi_axis, simulator, control_frequency, num_iterations):
    from numpy import random
    from boost_asteroid import boost_asteroid
    Asteroid = boost_asteroid.Asteroid

    asteroid = Asteroid(semi_axis, 2000.0, [0.1, 0.0, 0.1], 0.0)
    state = spacecraft_initialize_state(semi_axis)
    t = random.uniform(0.0, 24.0 * 60.0 * 60.0)
    dt = 1.0 / control_frequency
    positions = []
    heights = []

    for i in range(num_iterations):
        position = state[0:3]
        surface_point, _ = asteroid.nearest_point_on_surface_to_position(position)
        height = [p - s for p, s in zip(position, surface_point)]
        positions.append(state[0:3])
        heights.append(height)
        thrust = SPACECRAFT_ACTIONS[spacecraft_pi(state, w)]
        state = simulator.next_state(state, thrust, t)
        t += dt

    return positions, heights

def spacecraft_prepare_samples(num_samples, num_steps):
    from math import sqrt
    from numpy import random, array
    from utility import sample_sign, sample_point_outside_ellipsoid
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

    control_frequency = 10.0

    target_position = sample_point_outside_ellipsoid(semi_axis, 4.0)

    sensor_noise = 0.05
    perturbation_noise = 1e-7
    control_noise = 0.05

    simulator = Simulator(semi_axis, density, angular_velocity, time_bias,
                          target_position, control_frequency, sensor_noise, perturbation_noise, control_noise)
    simulator.init_spacecraft_specific_impulse(spacecraft_specific_impulse)

    for i in xrange(num_samples):
        state = spacecraft_initialize_state(semi_axis)
        t = random.uniform(0.0, 24.0 * 60.0 * 60.0)
        for j in xrange(num_steps):
            a = random.choice([0, 1, 2, 3, 4, 5, 6])
            thrust = SPACECRAFT_ACTIONS[a]
            position = state[0:3]
            next_state = simulator.next_state(state, thrust, t)
            t += 1.0 / control_frequency
            next_position = next_state[0:3]

            err_state = sqrt(sum([(p-np) ** 2 for p, np in zip(target_position, position)]))
            err_next_state = sqrt(sum([(p-np) ** 2 for p, np in zip(target_position, next_position)]))

            r = err_state - err_next_state
            if r < 0:
                r = 0
            else:
                r = -1.0

            sample = (array(state), a, r, array(next_state))
            samples.append(sample)
            state = next_state

    return samples, semi_axis, simulator, control_frequency