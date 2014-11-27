def plot_3d(positions):
    import matplotlib.pyplot as pyplot
    from mpl_toolkits.mplot3d import Axes3D

    fig = pyplot.figure()
    ax = fig.gca(projection="3d")
    ax.plot(positions[:, 0], positions[:, 1], positions[
            :, 2], label="spacecraft trajectory")
    pyplot.plot([positions[0][0]], [positions[0][1]],
                [positions[0][2]], 'rD', label="start")
    pyplot.plot([positions[-1][0]], [positions[-1][1]],
                [positions[-1][2]], 'bD', label="end")
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    pyplot.show()


def visualize(asteroid, positions, velocities, heights, velocities_vertical, velocities_remaining,
              accelerations_perturbations, accelerations_centrifugal, accelerations_coriolis, accelerations_euler,
              accelerations_gravity, frequency):
    from visual import ellipsoid, box, rate, color, vector, arrow, text, scene, sphere
    from numpy.linalg import norm

    scaling = 1e9
    scene.up = vector(0, 0, 1)

    asteroid_3d = ellipsoid(pos=(0.0, 0.0, 0.0),
                            length=2.0 * asteroid.semi_axis_a,
                            height=2.0 * asteroid.semi_axis_b,
                            width=2.0 * asteroid.semi_axis_c,
                            color=(153.0 / 255.0, 76.0 / 255.0, 0.0))

    spacecraft = box(pos=tuple(positions[0]), size=(100, 100, 100), make_trail=True,
                     color=(0.0, 128.0 / 255.0, 1.0))
    spacecraft.trail_object.color = color.gray(0.5)

    x = arrow(pos=(0, 0, 0), axis=(asteroid_3d.length + 200.0, 0.0, 0.0), shaftwidth=100.0, color=color.green)
    y = arrow(pos=(0, 0, 0), axis=(0.0, asteroid_3d.height + 200.0, 0.0), shaftwidth=100.0, color=color.red)
    z = arrow(pos=(0, 0, 0), axis=(0.0, 0.0, asteroid_3d.width + 200.0), shaftwidth=100.0, color=color.blue)

    text(text='x', axis=x.axis, pos=x.axis, height=250.0)
    text(text='y', axis=y.axis, pos=y.axis, height=250.0)
    text(text='z', axis=z.axis, pos=z.axis, height=250.0)

    norm_velocity = 1.0  # norm(velocities[0])
    velocity = arrow(pos=spacecraft.pos, axis=tuple([scaling * var / norm_velocity for var in velocities[0]]), shaftwidth=20.0,
                     color=color.magenta, visible=False)

    height = arrow(pos=spacecraft.pos-heights[0], axis=tuple(heights[0]), shaftwidth=20.0,
                   color=color.orange, visible=False)

    ground_position = sphere(pos=height.pos, radius=5.0, make_trail=True, color=(0.0, 0.0, 204.0 / 255.0))
    ground_position.trail_object.color = (25.0 / 255.0, 25.0 / 255.0, 112.0 / 255.0)

    velocity_vertical = arrow(pos=spacecraft.pos, axis=tuple(velocities_vertical[0]), shaftwidth=15.0,
                              color=color.cyan, visible=False)

    velocity_remaining = arrow(pos=spacecraft.pos, axis=tuple(velocities_remaining[0]), shaftwidth=15.0,
                              color=(250.0 / 255.0, 128.0 / 255.0, 114.0 / 255.0), visible=False)

    acceleration_coriolis = arrow(pos=spacecraft.pos, axis=tuple(accelerations_coriolis[0]), shaftwidth=15.0,
                                  color=color.green)

    acceleration_centrifugal = arrow(pos=spacecraft.pos, axis=tuple(accelerations_centrifugal[0]), shaftwidth=15.0,
                                  color=color.red)

    acceleration_euler = arrow(pos=spacecraft.pos, axis=tuple(accelerations_euler[0]), shaftwidth=15.0,
                                  color=color.blue)

    acceleration_perturbations = arrow(pos=spacecraft.pos, axis=tuple(accelerations_perturbations[0]), shaftwidth=15.0,
                                  color=color.yellow)

    acceleration_gravity = arrow(pos=spacecraft.pos, axis=tuple(accelerations_gravity[0]), shaftwidth=15.0,
                                  color=color.white)

    for i in range(1, len(positions)):
        rate(frequency)

        spacecraft.pos = tuple(positions[i])
        velocity.pos = spacecraft.pos

        norm_velocity = 1.0  # norm(velocities[i])
        velocity.axis = tuple([scaling * var / norm_velocity for var in velocities[i]])

        height.pos = spacecraft.pos-heights[i]
        height.axis = tuple(heights[i])
        ground_position.pos = height.pos

        norm_vel_vert = 1.0  # norm(velocities_vertical[i])
        velocity_vertical.pos = spacecraft.pos
        velocity_vertical.axis = tuple([scaling * var / norm_vel_vert for var in velocities_vertical[i]])

        norm_vel_rem = 1.0  # norm(velocities_remaining[i])
        velocity_remaining.pos = spacecraft.pos
        velocity_remaining.axis = tuple([scaling * var / norm_vel_rem for var in velocities_remaining[i]])

        norm_acc_cor = 1.0  # norm(accelerations_coriolis[i])
        acceleration_coriolis.pos = spacecraft.pos
        acceleration_coriolis.axis = tuple([scaling * var / norm_acc_cor for var in accelerations_coriolis[i]])

        norm_acc_cen = 1.0  # norm(accelerations_centrifugal[i])
        acceleration_centrifugal.pos = spacecraft.pos
        acceleration_centrifugal.axis = tuple([scaling * var / norm_acc_cen for var in accelerations_centrifugal[i]])

        norm_acc_eul = 1.0  # norm(accelerations_euler[i])
        acceleration_euler.pos = spacecraft.pos
        acceleration_euler.axis = tuple([scaling * var / norm_acc_eul for var in accelerations_euler[i]])

        norm_acc_per = 1.0  # norm(accelerations_perturbations[i])
        acceleration_perturbations.pos = spacecraft.pos
        acceleration_perturbations.axis = tuple([scaling * var / norm_acc_per for var in accelerations_perturbations[i]])

        norm_acc_grav = 1.0  # norm(accelerations_gravity[i])
        acceleration_gravity.pos = spacecraft.pos
        acceleration_gravity.axis = tuple([scaling * var / norm_acc_grav for var in accelerations_gravity[i]])