'''
usage:
./python2.7 vtrajectory.py <data_file> [speedup] [reference frame]

default speedup is 400. default reference frame is body.

examples:
./python2.7 vtrajectory.py trajectory.txt 400 body
./python2.7 vtrajectory.py trajectory.txt 1 inertial
./python2.7 vtrajectory.py trajectory.txt 123
./python2.7 vtrajectory.py trajectory.txt

'''

import sys
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid
from visual import ellipsoid, box, rate, color, vector, arrow, scene, sphere, label, display

from numpy.linalg import norm, det
from time import sleep
from numpy import array, eye, dot, linspace
from math import cos, sin
from scipy.integrate import odeint

speedup = 400
reference_frame = "body"

file_name = sys.argv[1]
if len(sys.argv) > 2:
    speedup = float(sys.argv[2])

if len(sys.argv) > 3:
    reference_frame = str(sys.argv[3])

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
num_samples = len(lines)
total_time = num_samples / frequency
states = [line.split(',') for line in lines]
for i in range(num_samples):
    for j in range(len(states[i])):
        states[i][j] = float(states[i][j])

result_file.close()

if reference_frame == "inertial":
    def d_dt_q(q, t):
        from math import isnan

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

scene.up = vector(0, 0, 1)

asteroid_3d = ellipsoid(pos=(0.0, 0.0, 0.0), up=(0.0, 0.0, 1.0),
                        length=2.0 * sim_params[0],
                        width=2.0 * sim_params[1],
                        height=2.0 * sim_params[2],
                        color=(153.0 / 255.0, 76.0 / 255.0, 0.0))

spacecraft = box(pos=tuple(states[0][0:3]), size=(100, 100, 100), make_trail=True,
                 color=(0.0, 128.0 / 255.0, 1.0), up=(0.0, 0.0, 1.0))
spacecraft.trail_object.color = color.gray(0.5)

height = arrow(pos=spacecraft.pos-states[0][3:7], axis=tuple(states[0][3:7]), shaftwidth=20.0,
               color=color.orange, up=(0.0, 0.0, 1.0))

if reference_frame == "body":
    ground_position = sphere(pos=height.pos, radius=5.0, make_trail=True, color=(0.0, 0.0, 204.0 / 255.0))
    ground_position.trail_object.color = (25.0 / 255.0, 25.0 / 255.0, 112.0 / 255.0)

else:
    ground_position = sphere(pos=height.pos, radius=5.0, color=(0.0, 0.0, 204.0 / 255.0))

norms = [asteroid_3d.length * 0.75, asteroid_3d.width * 0.75, asteroid_3d.height * 0.75]

if reference_frame == "body":
    x_axis = arrow(pos=(0, 0, 0), axis=(norms[0], 0.0, 0.0), shaftwidth=100.0, color=color.green)
    y_axis = arrow(pos=(0, 0, 0), axis=(0.0, norms[1], 0.0), shaftwidth=100.0, color=color.red)
    z_axis = arrow(pos=(0, 0, 0), axis=(0.0, 0.0, norms[2]), shaftwidth=100.0, color=color.blue)

else:
    x_axis = arrow(pos=(0, 0, 0), axis=(norms[0], 0.0, 0.0), shaftwidth=100.0, color=color.green)
    y_axis = arrow(pos=(0, 0, 0), axis=(0.0, norms[0], 0.0), shaftwidth=100.0, color=color.red)
    z_axis = arrow(pos=(0, 0, 0), axis=(0.0, 0.0, norms[0]), shaftwidth=100.0, color=color.blue)

x_axis_label = label(text='x', pos=x_axis.axis, zoffset=20.0)
y_axis_label = label(text='y', pos=y_axis.axis)
z_axis_label = label(text='z', pos=z_axis.axis, zoffset=20.0)

height_label = label(text='nan', pos=height.pos/2.0)



def q2av(q):
    from math import acos, sqrt

    axis = [0.0, 0.0, 0.0]
    q1x = q[0]
    q1y = q[1]
    q1z = q[2]
    q1w = q[3]
    angle = 2.0 * acos(q1w)
    axis[0] = q1x
    axis[1] = q1y
    axis[2] = q1z
    magnitude = sqrt(sum([val * val for val in axis]))
    axis = [val / magnitude for val in axis]
    return angle, tuple(axis)


previous_angle = 0.0
previous_axis = (0.0, 0.0, 0.0)

current_time = 0.0
time_window = display(title='time [s]', x=0, y=0, width=200, height=100)
time_label = label(text='0.0')

print("ready. click to start visualization...")
scene.waitfor('click')

dt =  1.0 / frequency
interval = speedup * frequency
for i in range(1, len(states)):
    rate(interval)

    if reference_frame == "body":
        spacecraft_pos = tuple(states[i][0:3])
        height_axis = tuple(states[i][3:7])

        spacecraft.pos = spacecraft_pos
        height.pos = spacecraft.pos - height_axis
        height.axis = height_axis
        ground_position.pos = height.pos
        height_label.pos = tuple([pos + val / 2.0 for pos, val in zip(ground_position.pos, height.axis)])
        height_label.text = str(norm(height.axis))

    else:
        spacecraft_pos = array(states[i][0:3])
        height_axis = array(states[i][3:7])

        current_rotation = rotation_matrices[i]
        current_quaternion = quaternions[i]

        spacecraft_pos = current_rotation.dot(spacecraft_pos)
        spacecraft.pos = tuple(spacecraft_pos)

        height_axis = tuple(current_rotation.dot(height_axis))
        height.pos = spacecraft.pos - height_axis
        height.axis = height_axis

        ground_position.pos = height.pos
        height_label.pos = tuple([pos + val / 2.0 for pos, val in zip(ground_position.pos, height.axis)])
        height_label.text = str(norm(height.axis))

        asteroid_3d.rotate(angle=-previous_angle, axis=previous_axis, origin=(0.0, 0.0, 0.0))

        previous_angle, previous_axis = q2av(current_quaternion)

        asteroid_3d.rotate(angle=previous_angle, axis=previous_axis, origin=(0.0, 0.0, 0.0))

    current_time += dt
    time_label.text = str(current_time)  

print("done.")