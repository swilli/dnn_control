import sys
from numpy import random
from numpy.linalg import norm
from scipy.integrate import odeint
from asteroid import Asteroid
from constants import PI
from numpy import array
from denoisingautoencoder import DenoisingAutoencoder
from test import neuralnetwork

'''
encoder = DenoisingAutoencoder(3, 2)

error = 0.0
num_tests = 100000
for i in range(num_tests):
    seed_1 = random.rand()
    seed_2 = random.rand()
    test_value = array([seed_1, seed_1, seed_1])
    error += norm(test_value - encoder.train(test_value))

print(error / num_tests)
'''

nn = neuralnetwork.NeuralNetwork()
print(0.5 - nn.forward_pass(array([0.35, 0.9])))
nn.train(array([0.35, 0.9]), 0.5)
print(0.5 - nn.forward_pass(array([0.35, 0.9])))

'''#todo write test for asteroid gravity, compare values

INERTIA_Z = 3000.0  # [kg*m^2]
INERTIA_Y = 2000.0  # [kg*m^2]
INERTIA_X = 1000.0  # [kg*m^2]
DENSITY = 2.3  # [kg/m^3]
ANGULAR_VELOCITY = [0.0005, 0.0, 0.0003]  # [1/s]
TIME_BIAS = 0.0  # [s]
asteroid = Asteroid(INERTIA_X, INERTIA_Y, INERTIA_Z, DENSITY, ANGULAR_VELOCITY, TIME_BIAS)


position = [0, 50, 0]
gravity = asteroid.gravity_at_position(position)
print(gravity)
'''
'''
def w_dot(state, time, inertia_x, inertia_y, inertia_z):
    return [(inertia_y - inertia_z) * state[1] * state[2] / inertia_x,
    (inertia_z - inertia_x) * state[2] * state[0] / inertia_y,
     (inertia_x - inertia_y) * state[0] * state[1] / inertia_z]

INERTIA_Z = 4567.123  # [kg*m^2]
INERTIA_Y = 2345.3456  # [kg*m^2]
INERTIA_X = 1234.12  # [kg*m^2]
DENSITY = 2000.0  # [kg/m^3]
ANGULAR_VELOCITY = [0.0005, 0.0, 0.0003]  # [1/s]
TIME_BIAS = 0.0  # [s]
asteroid = Asteroid(INERTIA_X, INERTIA_Y, INERTIA_Z, DENSITY, ANGULAR_VELOCITY, TIME_BIAS)


test_time = 8000

result = odeint(
    w_dot, ANGULAR_VELOCITY, [0, test_time], (INERTIA_X, INERTIA_Y, INERTIA_Z))
omega_odeint = result[1][:]
omega_analytical = asteroid.angular_velocity_at_time(test_time)


print("{0:.10f} {1:.10f} {2:.10f}".format(omega_odeint[0],omega_odeint[1],omega_odeint[2]))
print("{0:.10f} {1:.10f} {2:.10f}".format(omega_analytical[0],omega_analytical[1],omega_analytical[2]))

'''

'''
def w_dot(state, time, inertia_x, inertia_y, inertia_z):
    return [(inertia_y - inertia_z) * state[1] * state[2] / inertia_x,
            (inertia_z - inertia_x) * state[2] * state[0] / inertia_y,
            (inertia_x - inertia_y) * state[0] * state[1] / inertia_z]

min_error = sys.float_info.max
max_error = sys.float_info.min
avg_error = 0.0

#random.seed(0)
trials = 2000

for i in range(trials):
    # print("Test run {}".format(i + 1))

    I = []
    test_time = random.uniform(1.0, 2.0)
    [I.append(random.uniform(1000.0, 5000.0)) for i in range(3)]
    inertia_x, inertia_y, inertia_z = sorted(I)

    density = 2000.0
    angular_velocity = [random.uniform(-0.02 * PI, 0.02 * PI), 0.0, random.uniform(-0.02 * PI, 0.02 * PI)]
    
    asteroid = Asteroid(
        inertia_x, inertia_y, inertia_z, density, angular_velocity, 0.0)

    result = odeint(w_dot, angular_velocity, [0, test_time], (inertia_x, inertia_y, inertia_z), rtol=1e-12, atol=1e-12)
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

'''


'''
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
