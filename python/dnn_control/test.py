def unit_test_pca():
    data = []
    for i in range(20 * 5000):
        seed_1 = random.rand() / 2.1
        seed_2 = random.rand() / 2.1
        seed_3 = seed_1 + seed_2
        data.append([seed_1, seed_2, seed_3])

    data = array(data)
    pca = PCA(n_components=2)
    pca.fit(data)

    for i in range(100):
        seed_1 = random.rand() / 2.1
        seed_2 = random.rand() / 2.1
        test_case = array([seed_1, seed_2, seed_1 + seed_2])
        print(test_case)
        x_tilde = pca.transform(test_case)
        z = pca.inverse_transform(x_tilde)
        print(x_tilde)
        print(z)
        print(norm(test_case - z))


def unit_test_linear_regression():
    data = []
    num_samples = 50000
    for i in range(num_samples):
        seed_1 = random.rand() / 2.1
        seed_2 = random.rand() / 2.1
        test_case = [seed_1, seed_2, seed_1 + seed_2]
        data.append(test_case)
    data = array(data)


    compressed_data = column_stack((data[:, 0:2], ones((num_samples, 1))))

    x, residuals, rank, s = lstsq(compressed_data, data)
    print(x)


def unit_test_random_projection():
    from sklearn.random_projection import SparseRandomProjection
    from numpy import dot, array, squeeze
    from numpy.linalg import pinv

    data = []
    num_samples = 50000
    for i in range(num_samples):
        seed_1 = random.rand() / 2.1
        seed_2 = random.rand() / 2.1
        test_case = [seed_1, seed_2, seed_1 + seed_2]
        data.append(test_case)

    encoder = SparseRandomProjection(n_components=2)
    encoder.fit(data)

    projection = encoder.components_.todense()
    seed_1 = random.rand() / 2.1
    seed_2 = random.rand() / 2.1
    test_case = array([seed_1, seed_2, seed_1 + seed_2])
    print(test_case)
    z = dot(projection, test_case)
    z = asarray(z)
    z = squeeze(z)
    print(z)
    print(dot(pinv(projection),z))


def unit_test_autoencoder():
    #AUTOENCODER UNIT TEST
    data = []
    for i in range(20 * 60000):
        seed_1 = random.rand() / 2.1
        seed_2 = random.rand() / 2.1
        test_case = [seed_1, seed_2, seed_1 + seed_2]
        data.append(test_case)
    data = array(data)
    train_set_x = shared(asarray(data, dtype=theano_config.floatX), borrow=True)

    batch_size = 20

    input = matrix('x')
    index = lscalar()

    num_training_batches = train_set_x.get_value(borrow=True).shape[0] / batch_size

    encoder = DenoisingAutoencoder(3, 2, learning_rate=0.1, input=input, corruption_level=0.3)

    cost, updates = encoder.get_cost_updates()

    train = function([index], cost, updates=updates,
                        givens={input: train_set_x[index * batch_size: (index + 1) * batch_size]})

    mean_costs = 1.0
    for epoch in xrange(15):
        costs = []
        for batch_index in xrange(num_training_batches):
            costs.append(train(batch_index))

        mean_costs = mean(costs)
        print 'Training epoch %d, cost ' % epoch, mean_costs


    seed_1 = random.rand() / 2.1
    seed_2 = random.rand() / 2.1
    test_case = array([seed_1, seed_2, seed_1 + seed_2])
    print(test_case)
    z = encoder.decompress(encoder.compress(test_case))
    print(z)
    print(norm(test_case - z))

def unit_test_height():
    from numpy import random, mean
    from time import time
    from asteroid import Asteroid

    #ASTEROID HEIGHT METHOD UNIT TEST
    SEMI_AXIS_C = 1000.0  # [m]
    SEMI_AXIS_B = 2000.0  # [m]
    SEMI_AXIS_A = 3000.0  # [m]
    DENSITY = 2000.0  # [kg/m^3]
    ANGULAR_VELOCITY = [0.0005, 0.0, 0.0003]  # [1/s]
    TIME_BIAS = 0.0  # [s]
    asteroid = Asteroid(SEMI_AXIS_A, SEMI_AXIS_B, SEMI_AXIS_C, DENSITY, ANGULAR_VELOCITY, TIME_BIAS)

    def check(surface_point, position):
        from math import copysign

        value = surface_point[0] ** 2 / SEMI_AXIS_A ** 2 + surface_point[1] ** 2 / SEMI_AXIS_B ** 2 + surface_point[2] ** 2 / SEMI_AXIS_C ** 2 - 1.0
        #print("{0} {1}".format(surface_point, value))
        for i in range(3):
            sign_surf = copysign(1, surface_point[i])
            sign_pos = copysign(1, position[i])
            assert(sign_pos == sign_surf)
        assert(abs(value) < 1e-10)

    signs = [-1.0, 1.0]
    times = []
    for i in range(10000):
        x = random.choice(signs) * random.uniform(SEMI_AXIS_A, SEMI_AXIS_A*1000.0)
        y = random.choice(signs) * random.uniform(SEMI_AXIS_B, SEMI_AXIS_B*1000.0)
        z = random.choice(signs) * random.uniform(SEMI_AXIS_C, SEMI_AXIS_C*1000.0)

        position = [x, y, z]
        start = time()
        distance, surface_point = asteroid.distance_to_surface_at_position(position)
        times.append(time() - start)
        check(surface_point, position)

    print(mean(times))

def unit_test_angular_velocity():
    from sys import float_info
    from numpy import random
    from numpy.linalg import norm
    from asteroid import Asteroid
    from scipy.integrate import odeint
    from constants import PI

    def w_dot(state, time, inertia_x, inertia_y, inertia_z):
        return [(inertia_y - inertia_z) * state[1] * state[2] / inertia_x,
                (inertia_z - inertia_x) * state[2] * state[0] / inertia_y,
                (inertia_x - inertia_y) * state[0] * state[1] / inertia_z]

    min_error = float_info.max
    max_error = float_info.min
    avg_error = 0.0

    #random.seed(0)
    trials = 10000

    for i in range(trials):
        # print("Test run {}".format(i + 1))

        axis = []
        test_time = random.uniform(1.0, 100.0)
        [axis.append(random.uniform(1000.0, 5000.0)) for i in range(3)]
        semi_axis_c, semi_axis_b, semi_axis_a = sorted(axis)

        density = 2000.0
        angular_velocity = [random.uniform(-0.02 * PI, 0.02 * PI), 0.0, random.uniform(-0.02 * PI, 0.02 * PI)]

        asteroid = Asteroid(semi_axis_a, semi_axis_b, semi_axis_c, density, angular_velocity, 0.0)

        result = odeint(w_dot, angular_velocity, [0, test_time], (asteroid.inertia_x, asteroid.inertia_y,
                                                                  asteroid.inertia_z), rtol=1e-12, atol=1e-12)
        omega_numerical = result[1][:]
        omega_analytical = asteroid.angular_velocity_at_time(test_time)
        error = norm(omega_numerical - omega_analytical)

        avg_error += error
        if error < min_error:
            min_error = error
        elif error > max_error:
            max_error = error

    print("Min error: {}".format(min_error))
    print("Max error: {}".format(max_error))
    print("Avg error: {}".format(avg_error / trials))

def unit_test_angular_acceleration():
    from sys import float_info
    from numpy import random, array
    from numpy.linalg import norm
    from asteroid import Asteroid
    from constants import PI

    def w_dot(state, inertia_x, inertia_y, inertia_z):
        return [(inertia_y - inertia_z) * state[1] * state[2] / inertia_x,
                (inertia_z - inertia_x) * state[2] * state[0] / inertia_y,
                (inertia_x - inertia_y) * state[0] * state[1] / inertia_z]

    min_error = float_info.max
    max_error = float_info.min
    avg_error = 0.0

    #random.seed(0)
    trials = 10000

    for i in range(trials):
        # print("Test run {}".format(i + 1))

        axis = []
        test_time = random.uniform(1.0, 100.0)
        [axis.append(random.uniform(1000.0, 5000.0)) for i in range(3)]
        semi_axis_c, semi_axis_b, semi_axis_a = sorted(axis)

        density = 2000.0
        angular_velocity = [random.uniform(-0.02 * PI, 0.02 * PI), 0.0, random.uniform(-0.02 * PI, 0.02 * PI)]

        asteroid = Asteroid(semi_axis_a, semi_axis_b, semi_axis_c, density, angular_velocity, 0.0)

        angular_velocity = asteroid.angular_velocity_at_time(test_time)

        angular_acceleration_correct = array(w_dot(angular_velocity, asteroid.inertia_x, asteroid.inertia_y, asteroid.inertia_z))
        angular_acceleration_class = asteroid.angular_acceleration_at_time(test_time)

        error = norm(angular_acceleration_correct - angular_acceleration_class)

        avg_error += error
        if error < min_error:
            min_error = error
        elif error > max_error:
            max_error = error

    print("Min error: {}".format(min_error))
    print("Max error: {}".format(max_error))
    print("Avg error: {}".format(avg_error / trials))

def unit_test_gravity():
    from numpy import random, mean
    from math import copysign
    from asteroid import Asteroid
    from constants import PI
    signs = [-1.0, 1.0]

    test_cases = 50000

    for i in range(test_cases):
        axis = []
        [axis.append(random.uniform(1000.0, 5000.0)) for i in range(3)]
        semi_axis_c, semi_axis_b, semi_axis_a = sorted(axis)

        density = random.uniform(1500.0, 2500.0)
        angular_velocity = [random.uniform(-0.02 * PI, 0.02 * PI), 0.0, random.uniform(-0.02 * PI, 0.02 * PI)]

        asteroid = Asteroid(semi_axis_a, semi_axis_b, semi_axis_c, density, angular_velocity, 0.0)

        position = [random.choice(signs) * semi_axis_a * (1.1 + 10.0 * random.rand()),
                    random.choice(signs) * semi_axis_b * (1.1 + 10.0 * random.rand()),
                    random.choice(signs) * semi_axis_c * (1.1 + 10.0 * random.rand())]

        gravity = asteroid.gravity_at_position(position)

        for i in range(3):
            sign_pos = copysign(1, position[i])
            sign_grav = copysign(1, gravity[i])
            if sign_pos == sign_grav:
                print("a = {0}; b = {1}; c = {2}; rho = {3}; x = {4}';".format(semi_axis_a, semi_axis_b, semi_axis_c, density, position))
                print(gravity)
                exit()


#unit_test_autoencoder()
#unit_test_pca()
#unit_test_linear_regression()
#unit_test_height()
#unit_test_angular_velocity()
#unit_test_angular_acceleration()
unit_test_gravity()



'''
#ASTEROID ANGULAR VELOCITY VISUALIZATION

TIME = 100000.0
SAMPLING_FREQUENCY = 10.0
INERTIA_X = 4567.123  # [kg*m^2]
INERTIA_Y = 2345.3456  # [kg*m^2]
INERTIA_Z = 1234.12  # [kg*m^2]
DENSITY = 2000.0  # [kg/m^3]
ANGULAR_VELOCITY = [0.0005, 0.0007, -0.0003]  # [1/s]
TIME_BIAS = 0.0  # [s]

asteroid = Asteroid(INERTIA_X, INERTIA_Y, INERTIA_Z, DENSITY, ANGULAR_VELOCITY, TIME_BIAS)

iterations = int(TIME*SAMPLING_FREQUENCY)

angular_velocity = empty([iterations,3])
for i in range(iterations):
    time = i*1.0/SAMPLING_FREQUENCY
    omega = asteroid.angular_velocity_at_time(time)
    angular_velocity[i][:] = omega

# Visualize trajectory
fig = pyplot.figure()
ax = fig.gca(projection="3d")
ax.plot(angular_velocity[:,0],angular_velocity[:,1],angular_velocity[:,2], label="angular velocity")
pyplot.plot([angular_velocity[0][0]],[angular_velocity[0][1]],[angular_velocity[0][2]], 'rD', label="start")
pyplot.plot([angular_velocity[-1][0]],[angular_velocity[-1][1]],[angular_velocity[-1][2]], 'bD', label="end")
ax.legend()
ax.set_xlabel('Omega_x')
ax.set_ylabel('Omega_y')
ax.set_zlabel('Omega_z')
pyplot.show()
'''
