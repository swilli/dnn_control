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
        test_time = 26.257487718094691  # random.uniform(1.0, 100.0)
        [axis.append(random.uniform(1000.0, 5000.0)) for i in range(3)]
        semi_axis_a, semi_axis_b, semi_axis_c = [10000.0, 6000.0, 4000.0] #sorted(axis)

        density = 2215.0
        #angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0004), 0.0,
        #                   random.choice(signs) * random.uniform(0.0002, 0.0004)]

        angular_velocity = [0.0002, 0.0, 0.0008]

        asteroid = Asteroid(semi_axis_a, semi_axis_b, semi_axis_c, density, angular_velocity, 0.0)

        result = odeint(w_dot, angular_velocity, [0, test_time], (asteroid.inertia_x, asteroid.inertia_y,
                                                                  asteroid.inertia_z))
        omega_numerical = result[1][:]
        omega_analytical = asteroid.angular_velocity_and_acceleration_at_time(test_time)[0]
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

    def w_dot(state, inertia_x, inertia_y, inertia_z):
        return [(inertia_y - inertia_z) * state[1] * state[2] / inertia_x,
                (inertia_z - inertia_x) * state[2] * state[0] / inertia_y,
                (inertia_x - inertia_y) * state[0] * state[1] / inertia_z]

    signs = [-1.0, 1.0]

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
        angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0004), 0.0,
                            random.choice(signs) * random.uniform(0.0002, 0.0004)]

        asteroid = Asteroid(semi_axis_a, semi_axis_b, semi_axis_c, density, angular_velocity, 0.0)

        angular_velocity = asteroid.angular_velocity_and_acceleration_at_time(test_time)

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

def unit_test_gravity_direction():
    from numpy import random, array, mgrid, sin, cos
    from math import copysign, sqrt
    from boost_asteroid import boost_asteroid
    Asteroid = boost_asteroid.BoostAsteroid

    from utility import sample_positions_outside_ellipsoid
    from mayavi import mlab
    from constants import PI, GRAVITATIONAL_CONSTANT

    signs = [-1.0, 1.0]

    num_samples = 1000

    c_semi_axis = random.uniform(100.0, 8000.0)
    b_semi_axis_n = random.uniform(1.1, 2.0)
    a_semi_axis_n = random.uniform(1.1 * b_semi_axis_n, 4.0)
    semi_axis = [a_semi_axis_n * c_semi_axis, b_semi_axis_n * c_semi_axis, c_semi_axis]

    density = random.uniform(1500.0, 3000.0)
    angular_velocity = 0.85 * sqrt((GRAVITATIONAL_CONSTANT * 4.0/3.0 * PI * semi_axis[0] * semi_axis[1] * semi_axis[2]
                                    * density) / (semi_axis[0] * semi_axis[0] * semi_axis[0]))

    angular_velocity_xz = [random.choice(signs) * random.uniform(angular_velocity * 0.5, angular_velocity),
                           random.choice(signs) * random.uniform(angular_velocity * 0.5, angular_velocity)]

    asteroid = Asteroid(semi_axis, density, angular_velocity_xz, 0.0)

    positions = sample_positions_outside_ellipsoid(semi_axis, 1.1, 4.0, num_samples)
    gravities = [asteroid.gravity_acceleration_at_position(pos) for pos in positions]

    positions = array(positions)
    gravities = array(gravities)
    for i in range(num_samples):
        pos = positions[i]
        grav = gravities[i]
        for j in range(3):
            sign_pos = copysign(1.0, pos[j])
            sign_grav = copysign(1.0, grav[j])
            if sign_pos == sign_grav:
                print("BUG")
                assert(False)

    mlab.figure()
    '''x,y,z = mgrid[positions[:, 0].min():positions[:, 0].max():100j,
            positions[:, 1].min():positions[:, 1].max():100j, positions[:, 2].min():positions[:, 2].max():100j]
    f = (x**2/semi_axis_a**2 + y**2/semi_axis_b**2 + z**2/semi_axis_c**2 - 1.0)

    ellipsoid = mlab.contour3d(f, contours=[0], extent=[-1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
    '''
    phi, theta = mgrid[0:PI:100j, 0:2.0*PI:100j]
    x = semi_axis[0] * sin(phi) * cos(theta)
    y = semi_axis[1] * sin(phi) * sin(theta)
    z = semi_axis[2] * cos(phi)
    mlab.mesh(x, y, z, representation="wireframe", line_width=0.5, color=(153.0/255.0, 76.0/255.0, 0.0))
    field = mlab.quiver3d(positions[:, 0], positions[:, 1], positions[:, 2],
                          gravities[:, 0], gravities[:, 1], gravities[:, 2], mode="arrow")

    #title_start = "a = {0} b = {1} c = {2} rho = {3}".format(semi_axis[0], semi_axis[1], semi_axis[2], density)

    #mlab.title(title_start)
    #mlab.axes(color=(.7, .7, .7), ranges=(positions[:, 0].min(), positions[:, 0].max(),
    #                                      positions[:, 1].min(), positions[:, 1].max(),
    #                                      positions[:, 2].min(), positions[:, 2].max()),
    #          xlabel='x', ylabel='y', zlabel='z',
    #          x_axis_visibility=True, z_axis_visibility=True, y_axis_visibility=True)
    mlab.show()


def unit_test_gravity_contour():
    from numpy import random, linspace, meshgrid, array
    from numpy.linalg import norm
    from utility import sample_positions_outside_ellipse
    from boost_asteroid import boost_asteroid
    Asteroid = boost_asteroid.BoostAsteroid
    from matplotlib.pyplot import imshow, scatter, colorbar, title, close, gcf, xlabel, ylabel
    from scipy.interpolate import Rbf

    signs = [-1.0, 1.0]

    semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]

    density = random.uniform(1500.0, 3000.0)
    angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0008),
                        random.choice(signs) * random.uniform(0.0002, 0.0008)]

    asteroid = Asteroid(semi_axis, density, angular_velocity, 0.0)

    num_samples = 1000
    title_start = "a = {0} b = {1} c = {2} \nrho = {3} plane = ".format(semi_axis[0], semi_axis[1], semi_axis[2],
                                                                        density)
    for dim in range(3):
        if dim == 0:
            plane = "yz"
            samples = sample_positions_outside_ellipse([semi_axis[1], semi_axis[2]], 1.1, 4.0, num_samples)
            samples = [[0.0] + pos for pos in samples]
            samples = array(samples)
            x = samples[:, 1]
            y = samples[:, 2]
        elif dim == 1:
            plane = "xz"
            samples = sample_positions_outside_ellipse([semi_axis[0], semi_axis[2]], 1.1, 4.0, num_samples)
            samples = [[pos[0]] + [0.0] + [pos[1]] for pos in samples]
            samples = array(samples)
            x = samples[:, 0]
            y = samples[:, 2]
        else:
            plane = "xy"
            samples = sample_positions_outside_ellipse([semi_axis[0], semi_axis[1]], 1.1, 4.0, num_samples)
            samples = [pos + [0.0] for pos in samples]
            samples = array(samples)
            x = samples[:, 0]
            y = samples[:, 1]

        z = array([norm(asteroid.gravity_acceleration_at_position(pos.tolist())) for pos in samples])

        # Set up a regular grid of interpolation points
        xi, yi = linspace(x.min(), x.max(), 100), linspace(y.min(), y.max(), 100)
        xi, yi = meshgrid(xi, yi)

        # Interpolate
        rbf = Rbf(x, y, z, function='linear')
        zi = rbf(xi, yi)

        imshow(zi, vmin=z.min(), vmax=z.max(), origin='lower',
                   extent=[x.min(), x.max(), y.min(), y.max()])
        scatter(x, y, c=z)
        colorbar()

        xlabel(plane[:1])
        ylabel(plane[1:])
        title(title_start + plane)
        img = gcf()
        img.savefig("gravity_plane_{0}.svg".format(plane))
        close()

def unit_test_gravity_speed():
    from numpy import random
    from asteroid import Asteroid
    from sys import float_info
    from time import time
    from utility import sample_positions_outside_ellipsoid

    signs = [-1.0, 1.0]

    semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]

    density = random.uniform(1500.0, 2500.0)
    angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0004), 0.0,
                        random.choice(signs) * random.uniform(0.0002, 0.0004)]

    asteroid = Asteroid(semi_axis_a, semi_axis_b, semi_axis_c, density, angular_velocity, 0.0)

    min_time = float_info.max
    max_time = float_info.min
    avg_time = 0.0

    num_samples = 20000
    band_width = 2000.0
    samples = sample_positions_outside_ellipsoid(semi_axis_a, semi_axis_b, semi_axis_c, band_width, num_samples)
    for i in xrange(num_samples):
        start = time()
        gravity = asteroid.gravity_at_position(samples[i])
        end = time()
        duration = end-start
        avg_time += duration

        if duration < min_time:
            min_time = duration
        elif duration > max_time:
            max_time = duration

    print("min: {0}".format(min_time))
    print("max: {0}".format(max_time))
    print("avg: {0}".format(avg_time / num_samples))


def unit_test_angular_velocity_period():
    from numpy import random, linspace, array
    from boost_asteroid import boost_asteroid
    Asteroid = boost_asteroid.BoostAsteroid
    from matplotlib.pyplot import plot, xlabel, ylabel, title, gcf, close

    signs = [-1.0, 1.0]

    semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]

    density = random.uniform(1500.0, 3000.0)
    angular_velocity_xz = [random.choice(signs) * random.uniform(0.0002, 0.0004),
                        random.choice(signs) * random.uniform(0.0002, 0.0004)]

    asteroid = Asteroid(semi_axis, density, angular_velocity_xz, 0.0)

    sample_points = linspace(0.0, 24.0 * 60.0 * 60.0, 1000)
    samples = array([asteroid.angular_velocity_and_acceleration_at_time(time)[0] for time in sample_points])

    title_start = "a = {0} b = {1} c = {2} \nrho = {3} plane = ".format(semi_axis[0], semi_axis[1], semi_axis[2],
                                                                        density)

    for dim in range(3):
        if dim == 0:
            plane = "tx"
            plot(sample_points, samples[:, 0])
        elif dim == 1:
            plane = "ty"
            plot(sample_points, samples[:, 1])
        else:
            plane = "tz"
            plot(sample_points, samples[:, 2])

        xlabel(plane[:1])
        ylabel(plane[1:])
        title(title_start + plane)
        img = gcf()
        img.savefig("omega_plane_{0}.svg".format(plane))
        close()

def unit_test_boost_asteroid():
    from time import time
    from math import sqrt
    from boost_asteroid import boost_asteroid
    from boost_simulator.utility import sample_point_outside_ellipsoid

    from asteroid import Asteroid as PythonAsteroid
    from numpy import random

    signs = [-1.0, 1.0]

    c_axis_n = 1.0
    band_width_scale = 100.0

    def check(surface_point, position, semi_axis):
        from math import copysign

        value = surface_point[0] ** 2 / semi_axis[0] ** 2 + surface_point[1] ** 2 / semi_axis[1] ** 2 \
                + surface_point[2] ** 2 / semi_axis[2] ** 2 - 1.0
        for i in range(3):
            sign_surf = copysign(1, surface_point[i])
            sign_pos = copysign(1, position[i])
            assert(sign_pos == sign_surf)
        assert(abs(value) < 1e-10)

    avg_grav = 0.0
    avg_omg = 0.0
    avg_domg = 0.0
    avg_srf = 0.0
    avg_grav_impl2 = 0.0
    time_boost = 0.0
    time_python = 0.0
    time_python_impl2 = 0.0
    num_tests = 100000
    for i in xrange(num_tests):
        b_axis_n = random.uniform(1.1, 2.0)
        a_axis_n = random.uniform(1.1 * b_axis_n, 4.0)
        distance = random.uniform(100.0, 8000.0)
        semi_axis = [a_axis_n * distance, b_axis_n * distance, c_axis_n * distance]  # [m]
        density = random.uniform(1500.0, 3000.0)  # [kg/m^3]
        angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0008),
                            0.0,
                            random.choice(signs) * random.uniform(0.0002, 0.0008)]  # [1/s]

        time_bias = 0.0  # random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]
        t = random.uniform(0.0, 1000.0)

        boost_ast = boost_asteroid.BoostAsteroid(semi_axis, density, angular_velocity, time_bias)
        python_asteroid = PythonAsteroid(semi_axis, density, angular_velocity, time_bias)
        position = sample_point_outside_ellipsoid(semi_axis, band_width_scale)

        start = time()
        grav_boost = boost_ast.gravity_at_position(position)
        end = time()
        time_boost += end - start
        start = time()
        grav_python = python_asteroid.gravity_at_position(position)
        end = time()
        time_python += end - start
        start = time()
        grav_python_impl2 = python_asteroid.gravity_at_position2(position)
        end = time()
        time_python_impl2 += end - start
        omg_boost, domg_boost = boost_ast.angular_velocity_and_acceleration_at_time(t)
        omg_python, domg_python = python_asteroid.angular_velocity_and_acceleration_at_time(t)
        srf_boost = boost_ast.nearest_point_on_surface_to_position(position)[0]
        srf_python = python_asteroid.nearest_point_on_surface_to_position(position)[0]
        check(srf_python, position, semi_axis)
        check(srf_boost, position, semi_axis)
        avg_grav += sqrt(sum([(b-p)*(b-p) for b,p in zip(grav_boost, grav_python)]))
        avg_grav_impl2 += sqrt(sum([(b-p)*(b-p) for b,p in zip(grav_python_impl2, grav_python)]))
        avg_srf += sqrt(sum([(b-p)*(b-p) for b,p in zip(srf_boost, srf_python)]))
        avg_omg += sqrt(sum([(b-p)*(b-p) for b,p in zip(omg_boost, omg_python)]))
        avg_domg += sqrt(sum([(b-p)*(b-p) for b,p in zip(domg_boost, domg_python)]))

    avg_grav /= num_tests
    avg_srf /= num_tests
    avg_omg /= num_tests
    avg_domg /= num_tests
    avg_grav_impl2 /= num_tests
    time_boost /= num_tests
    time_python_impl2 /= num_tests
    time_python /= num_tests
    print("gravity : {0}".format(avg_grav))
    print("surface : {0}".format(avg_srf))
    print("omega : {0}".format(avg_omg))
    print("domega : {0}".format(avg_domg))
    print("grav impl2 : {0}".format(avg_grav_impl2))
    print("time boost : {0}".format(time_boost))
    print("time python : {0}".format(time_python))
    print("time python impl2 : {0}".format(time_python_impl2))





#unit_test_autoencoder()
#unit_test_pca()
#unit_test_linear_regression()
#unit_test_height()
#unit_test_angular_velocity()
#unit_test_angular_acceleration()
unit_test_gravity_direction()
#unit_test_gravity_contour()
#unit_test_gravity_speed()
#unit_test_angular_velocity_period()
#unit_test_boost_asteroid()

'''from asteroid import Asteroid
semi_axis = [5000.0, 2567.0, 1235.0]
density = 2215.0
angular_velocity = [-0.0002, 0.0, 0.0008]
time_bias = 0.0
ast = Asteroid(semi_axis, density, angular_velocity, time_bias)

position = [6789000.123, 3456000.123, 2345000.987]
surface_position, distance = ast.nearest_point_on_surface_to_position(position)
print(surface_position)
'''