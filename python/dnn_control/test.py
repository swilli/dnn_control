import matplotlib
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from asteroid import *

TIME = 100000.0
SAMPLING_FREQUENCY = 10.0

INERTIA_X = 4567.0 # [m]
INERTIA_Y = 2345.0 # [m]
INERTIA_Z = 1234.0 # [m]
DENSITY = 2000.0 # [kg/m^3]
ANGULAR_VELOCITY = [0.00029565148,0.00029565148,0.00029565148] # [1/s]
TIME_BIAS = 0.0 # [s]

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
