import matplotlib
import numpy 
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D

'''
mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
z = np.linspace(-2, 2, 100)
r = z**2 + 1
x = r * np.sin(theta)
y = r * np.cos(theta)
ax.plot(x, y, z, label='parametric curve')
ax.legend()

plt.show()
'''


import constants
import pidcontroller
import sensorsimulator
import simulator



# Simulation settings
TIME = 100 # [s]

# Asteroid settings
C_SEMI_AXIS = 4000 # [m]
B_SEMI_AXIS = 1000 # [m]
A_SEMI_AXIS = 500 # [m]
DENSITY = 2000 # [kg/m^3]
THETA = 0#constants.PI/2
OMEGA_ZERO = 0.1

# Spacecraft settings
POSITION = [1e3,1e3,1e3] # [m]
VELOCITY = [0,0,0] # [m/s]
MASS = 1000 # [kg]
SPECIFIC_IMPULSE = 200

# Controller settings
CONTROL_FREQUENCY = 10 # [Hz]

# Instantiate sensor simulator
sensor_simulator = sensorsimulator.SensorSimulator()
# Instantiate controller
controller = pidcontroller.PIDController()
# Instantiate simulator
simulator = simulator.Simulator(A_SEMI_AXIS, B_SEMI_AXIS, C_SEMI_AXIS, DENSITY, THETA, OMEGA_ZERO, POSITION, VELOCITY, MASS, SPECIFIC_IMPULSE, sensor_simulator, controller, CONTROL_FREQUENCY)

# Run simulator
positions = simulator.run(TIME, True)

# Visualize trajectory
fig = pyplot.figure()
ax = fig.gca(projection="3d")
ax.plot(positions[:,0],positions[:,1],positions[:,2], label="spacecraft trajectory")
ax.legend()
pyplot.plot([positions[0][0]],[positions[0][1]],[positions[0][2]], 'rD')
pyplot.plot([positions[-1][0]],[positions[-1][1]],[positions[-1][2]], 'bD')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
pyplot.show()
