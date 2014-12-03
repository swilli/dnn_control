import sys
from visual import ellipsoid, box, rate, color, vector, arrow, scene, sphere, label
from numpy.linalg import norm
from time import sleep

speedup = 400

file_name = sys.argv[1]
if len(sys.argv) > 2:
    speedup = float(sys.argv[2])

result_file = open(file_name, 'r')
sim_params = [float(value) for value in result_file.readline().split(',')]
frequency = sim_params[3]
lines = result_file.readlines()
states = [line.split(',') for line in lines]
for i in range(len(states)):
    for j in range(len(states[i])):
        states[i][j] = float(states[i][j])

result_file.close()

scene.up = vector(0, 0, 1)

asteroid_3d = ellipsoid(pos=(0.0, 0.0, 0.0),
                        length=2.0 * sim_params[0],
                        height=2.0 * sim_params[1],
                        width=2.0 * sim_params[2],
                        color=(153.0 / 255.0, 76.0 / 255.0, 0.0))

spacecraft = box(pos=tuple(states[0][0:3]), size=(100, 100, 100), make_trail=True,
                 color=(0.0, 128.0 / 255.0, 1.0))
spacecraft.trail_object.color = color.gray(0.5)

height = arrow(pos=spacecraft.pos-states[0][3:7], axis=tuple(states[0][3:7]), shaftwidth=20.0,
               color=color.orange)

ground_position = sphere(pos=height.pos, radius=5.0, make_trail=True, color=(0.0, 0.0, 204.0 / 255.0))
ground_position.trail_object.color = (25.0 / 255.0, 25.0 / 255.0, 112.0 / 255.0)

norms = [asteroid_3d.length + 200.0, asteroid_3d.height + 200.0, asteroid_3d.width + 200.0]

x = arrow(pos=(0, 0, 0), axis=(norms[0], 0.0, 0.0), shaftwidth=100.0, color=color.green)
y = arrow(pos=(0, 0, 0), axis=(0.0, norms[1], 0.0), shaftwidth=100.0, color=color.red)
z = arrow(pos=(0, 0, 0), axis=(0.0, 0.0, norms[2]), shaftwidth=100.0, color=color.blue)

label(text='x', pos=x.axis, zoffset=20.0)
label(text='y', pos=y.axis)
label(text='z', pos=z.axis, zoffset=20.0)
height_label = label(text='nan', pos=height.pos/2.0)

for i in range(1, len(states)):
    rate(speedup * frequency)

    spacecraft.pos = tuple(states[i][0:3])
    height.pos = spacecraft.pos-states[i][3:7]
    height.axis = tuple(states[i][3:7])
    ground_position.pos = height.pos
    height_label.pos = tuple([pos + val / 2.0 for pos, val in zip(ground_position.pos, height.axis)])
    height_label.text = str(norm(spacecraft.pos))



print("done.")