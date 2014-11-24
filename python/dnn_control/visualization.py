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


def visualize(asteroid, positions, frequency):
    from visual import ellipsoid, box, rate, color, vector, arrow, text, scene

    scene.up = vector(0, 0, 1)

    asteroid_3d = ellipsoid(pos=(0.0, 0.0, 0.0),
                            length=2.0 * asteroid.semi_axis_a,
                            height=2.0 * asteroid.semi_axis_b,
                            width=2.0 * asteroid.semi_axis_c,
                            color=(153.0 / 255.0, 76.0 / 255.0, 0.0))

    spacecraft = box(pos=tuple(positions[0]), size=(100, 100, 100), make_trail=True,
                     color=(0.0, 128.0 / 255.0, 1.0))
    spacecraft.trail_object.color = color.yellow

    x = arrow(pos=(0, 0, 0), axis=(asteroid_3d.length + 200.0, 0.0, 0.0), shaftwidth=100.0, color=color.green)
    y = arrow(pos=(0, 0, 0), axis=(0.0, asteroid_3d.height + 200.0, 0.0), shaftwidth=100.0, color=color.red)
    z = arrow(pos=(0, 0, 0), axis=(0.0, 0.0, asteroid_3d.width + 200.0), shaftwidth=100.0, color=color.blue)

    text(text='x', axis=x.axis, pos=x.axis, height=250.0)
    text(text='y', axis=y.axis, pos=y.axis, height=250.0)
    text(text='z', axis=z.axis, pos=z.axis, height=250.0)


    for i in range(1, len(positions)):
        rate(frequency)
        spacecraft.pos = tuple(positions[i])

