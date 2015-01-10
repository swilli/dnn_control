# Spacecraft LSPI

SPACECRAFT_STATE_DIMENSION = 10
SPACECRAFT_MAXIMUM_THRUST = 50.0

action_index = 0
SPACECRAFT_ACTIONS = {}
for action_x in [-SPACECRAFT_MAXIMUM_THRUST, 0.0, SPACECRAFT_MAXIMUM_THRUST]:
    for action_y in [-SPACECRAFT_MAXIMUM_THRUST, 0.0, SPACECRAFT_MAXIMUM_THRUST]:
        for action_z in [-SPACECRAFT_MAXIMUM_THRUST, 0.0, SPACECRAFT_MAXIMUM_THRUST]:
            SPACECRAFT_ACTIONS[action_index] = [action_x, action_y, action_z]
            action_index += 1

SPACECRAFT_POLYNOMIAL_DIMENSIONS = int(0.5 * (3 * SPACECRAFT_STATE_DIMENSION ** 2 + SPACECRAFT_STATE_DIMENSION + 2))

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


def spacecraft_initialize_state(target_position, asteroid):
    from numpy import random
    from utility import sample_point_outside_ellipsoid

    position = [random.uniform(-3.0 + pos, 3.0 + pos) for pos in target_position]
    velocity = [random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), random.uniform(-0.3, 3.0)]
    mass = random.uniform(450.0, 550.0)

    t = random.uniform(0.0, 6.0 * 60.0 * 60.0)
    angular_velocity, _ = asteroid.angular_velocity_and_acceleration_at_time(t)

    return position + velocity + [mass] + angular_velocity, t


def spacecraft_pi(s, w):
    from numpy import random
    from numpy import dot
    from sys import float_info

    best_a = [random.choice([0, 1, 2, 3, 4, 5, 6])]
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
    Asteroid = boost_asteroid.BoostAsteroid

    asteroid = Asteroid(semi_axis, 2000.0, [0.1, 0.1], 0.0)
    state = spacecraft_initialize_state(semi_axis) + [random.uniform(0.0, 24.0 * 60.0 * 60.0)]

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
        state = simulator.next_state(state, thrust, state[7]) + [state[7] + dt]

    return positions, heights

def spacecraft_prepare_samples(num_samples, num_steps):
    from math import sqrt
    from numpy import random, array
    from utility import sample_sign, sample_point_outside_ellipsoid
    from boost_asteroid import boost_asteroid
    Asteroid = boost_asteroid.BoostAsteroid
    from boost_simulator import boost_simulator
    Simulator = boost_simulator.BoostSimulator

    samples = []

    semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]
    density = random.uniform(1500.0, 3000.0)
    angular_velocity_xz = [sample_sign() * random.uniform(0.0002, 0.0008), sample_sign() * random.uniform(0.0002, 0.0008)]
    time_bias = random.uniform(0.0, 6.0 * 60.0 * 60.0)

    asteroid = Asteroid(semi_axis, density, angular_velocity_xz, time_bias)

    spacecraft_specific_impulse = 200.0
    spacecraft_maximum_thrust = SPACECRAFT_MAXIMUM_THRUST

    control_frequency = 10.0
    full_state_controlled = False
    controlling_type = "none"

    target_position = [0.0, 0.0, 0.0]

    sensor_noise = [0.05]
    perturbation_noise = 1e-7
    control_noise = 0.05

    simulator = Simulator(semi_axis, density, angular_velocity_xz, time_bias,
                          full_state_controlled, controlling_type, target_position, spacecraft_maximum_thrust, control_frequency,
                          sensor_noise, perturbation_noise, control_noise)

    simulator.init_spacecraft_specific_impulse(spacecraft_specific_impulse)

    target_position = sample_point_outside_ellipsoid(asteroid.semi_axis(), 4.0)
    dt = 1.0 / control_frequency
    for i in xrange(num_samples):
        state, t = spacecraft_initialize_state(target_position, asteroid)

        for j in xrange(num_steps):
            a = random.randint(0, len(SPACECRAFT_ACTIONS))
            thrust = SPACECRAFT_ACTIONS[a]
            position = state[0:3]
            next_state = simulator.next_state(state[0:7], thrust, t) + asteroid.angular_velocity_and_acceleration_at_time(t + dt)[0]
            next_position = next_state[0:3]

            err_state = sqrt(sum([(p-np) ** 2 for p, np in zip(target_position, position)]))
            err_next_state = sqrt(sum([(p-np) ** 2 for p, np in zip(target_position, next_position)]))

            r = err_state - err_next_state

            sample = (array(state), a, r, array(next_state))
            samples.append(sample)
            state = next_state
            t += dt

    return samples, asteroid, simulator