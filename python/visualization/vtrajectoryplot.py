'''
usage:
./python2.7 vtrajectoryplot.py <data_file> [reference frame]

default reference frame is inertial.

examples:
./python2.7 vtrajectoryplot.py trajectory.txt body
./python2.7 vtrajectoryplot.py trajectory.txt inertial
./python2.7 vtrajectoryplot.py trajectory.txt

'''

import sys
import numpy as np
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from numpy import array, linspace
from scipy.integrate import odeint

reference_frame = "inertial"

file_name = sys.argv[1]
if len(sys.argv) > 2:
    reference_frame = str(sys.argv[2])

print("preparing data... ")

result_file = open(file_name, 'r')

sim_params = [float(value) for value in result_file.readline().split(',')]
frequency = sim_params[7]
semi_axis = sim_params[0:3]
density = sim_params[3]
angular_velocity_xz = [sim_params[4], sim_params[5]]
time_bias = sim_params[6]

asteroid = Asteroid(semi_axis, density, angular_velocity_xz, time_bias)

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
total_time = num_samples / frequency
print(total_time)
states = [line.split(',') for line in lines]
states = array([[float(value) for value in line] for line in states])

positions = states[:, 0:3]

if reference_frame == "inertial":
    def d_dt_q(q, t):

        Q = 0.5 * array([[q[3], -q[2], q[1]],
                         [q[2], q[3], -q[0]],
                         [-q[1], q[0], q[3]],
                         [-q[0], -q[1], -q[2]]])

        w, _ = array(asteroid.angular_velocity_and_acceleration_at_time(t))
        q_dot = Q.dot(w).tolist()
        return q_dot

    q0 = [0.0, 0.0, 0.0, 1.0]
    t = linspace(0.0, total_time, num_samples)
    quaternions = odeint(d_dt_q, q0, t)
    rotation_matrices = [array([[q[3] ** 2 + q[0] ** 2 - q[1] ** 2 - q[2] ** 2, 2.0 * (q[0] * q[1] - q[3] * q[2]), 2.0 * (q[0] * q[2] + q[3] * q[1])],
                                [2.0 * (q[0] * q[1] + q[3] * q[2]), q[3] ** 2 - q[0] ** 2 + q[1] ** 2 - q[2] ** 2, 2.0 * (q[1] * q[2] - q[3] * q[0])],
                                [2.0 * (q[0] * q[2] - q[3] * q[1]), 2.0 * (q[1] * q[2] + q[3] * q[0]), q[3] ** 2 - q[0] ** 2 - q[1] ** 2 + q[2] ** 2]]) for q in quaternions]

    positions = array([rot.dot(pos) for rot, pos in zip(rotation_matrices, positions)])




fig = plt.figure()
ax = fig.gca(projection='3d')
# Set of all spherical angles:
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)

# Cartesian coordinates that correspond to the spherical angles:
# (this is the equation of an ellipsoid):
x = semi_axis[0] * np.outer(np.cos(u), np.sin(v))
y = semi_axis[1] * np.outer(np.sin(u), np.sin(v))
z = semi_axis[2] * np.outer(np.ones_like(u), np.cos(v))

# Plot:
ax.plot_surface(x, y, z,  rstride=2, cstride=2, color='#994c00', alpha=1.0)
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="Spacecraft trajectory")
ax.plot([positions[0,0]],[positions[0,1]], [positions[0,2]], 'go', label="Start")
ax.plot([positions[-1,0]], [positions[-1,1]], [positions[-1,2]], 'ro', label="End")
ax.legend()
min_x = ax.get_xlim()[1]
min_y = ax.get_ylim()[1]
min_z = ax.get_zlim()[1]
max_dim = max(min_x, max(min_y, min_z))
ax.set_xlim([-max_dim, max_dim])
ax.set_ylim([-max_dim, max_dim])
ax.set_zlim([-max_dim, max_dim])
plt.show()




