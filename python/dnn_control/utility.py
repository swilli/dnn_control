# returns u x v
def cross_product(u, v):
    return [u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]]


# returns -1.0 or 1.0, each with probability 1/2
def sample_sign():
    from numpy import random

    return random.choice([-1.0, 1.0])


def sample_point_outside_ellipsoid(semi_axis, min_scale, max_scale):
    from numpy import random
    from math import cos, sin
    from constants import PI

    u = random.uniform(0.0, 2.0 * PI)
    v = random.uniform(0.0, PI)
    return [random.uniform(min_scale, max_scale) * semi_axis[0] * cos(u) * sin(v),
            random.uniform(min_scale, max_scale) * semi_axis[1] * sin(u) * sin(v),
            random.uniform(min_scale, max_scale) * semi_axis[2] * cos(v)]


def sample_point_outside_ellipse(semi_axis, min_scale, max_scale):
    from numpy import random
    from math import cos, sin
    from constants import PI

    theta = random.uniform(0.0, 2.0*PI)
    return [random.uniform(min_scale, max_scale) * semi_axis[0] * cos(theta),
            random.uniform(min_scale, max_scale) * semi_axis[1] * sin(theta)]


def sample_positions_outside_ellipse(semi_axis, min_scale, max_scale, num_samples):
    samples = []

    for i in range(num_samples):
        samples.append(sample_point_outside_ellipse(semi_axis, min_scale, max_scale))

    return samples


def sample_positions_outside_ellipsoid(semi_axis, min_scale, max_scale, num_samples):
    samples = []

    for i in range(num_samples):
        samples.append(sample_point_outside_ellipsoid(semi_axis, min_scale,max_scale))

    return samples