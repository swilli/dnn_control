# returns u x v
def cross_product(u, v):
    return [u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]]

# returns -1.0 or 1.0, each with probability 1/2
def sample_sign():
    from numpy import random

    return random.choice([-1.0, 1.0])

# Returns a point, whereas
# semi_axis_[0] * cos(u) * sin(v) < point[0] < semi_axis[0] * band_width_scale * cos(u) * sin(v)
# semi_axis_[1] * sin(u) * sin(v) < point[1] < semi_axis[1] * band_width_scale * sin(u) * sin(v)
# semi_axis_[2] * cos(v)          < point[2] < semi_axis[1] * band_width_scale * cos(v)
def sample_point_outside_ellipsoid(semi_axis, band_width_scale):
    from numpy import random
    from math import cos, sin
    from constants import PI

    u = random.uniform(0.0, 2.0 * PI)
    v = random.uniform(0.0, PI)
    return [random.uniform(1.0, band_width_scale) * semi_axis[0] * cos(u) * sin(v),
            random.uniform(1.0, band_width_scale) * semi_axis[1] * sin(u) * sin(v),
            random.uniform(1.0, band_width_scale) * semi_axis[2] * cos(v)]


# Returns num_samples of points, whereas for all i
# semi_axis_a * cos(theta) < point[i][0] < semi_axis_a * band_width_scale * cos(theta)
# semi_axis_b * sin(theta) < point[i][1] < semi_axis_b * band_width_scale * sin(theta)
def sample_positions_outside_ellipse(semi_axis_a, semi_axis_b, band_width_scale, num_samples):
    from numpy import random
    from math import cos, sin
    from constants import PI

    samples = []

    for i in range(num_samples):
        theta = random.uniform(0.0, 2.0*PI)
        x = random.uniform(1.0, band_width_scale) * semi_axis_a * cos(theta)
        y = random.uniform(1.0, band_width_scale) * semi_axis_b * sin(theta)
        samples.append([x, y])

    return samples


# Returns num_samples of points, whereas for all i
# semi_axis_a * cos(u) * sin(v) < point[i][0] < semi_axis_a * band_width_scale * cos(u) * sin(v)
# semi_axis_b * sin(u) * sin(v) < point[i][1] < semi_axis_b * band_width_scale * sin(u) * sin(v)
# semi_axis_c * cos(v)          < point[i][2] < semi_axis_c * band_width_scale * cos(v)
def sample_positions_outside_ellipsoid(semi_axis_a, semi_axis_b, semi_axis_c, band_width_scale, num_samples):
    from numpy import random
    from math import cos, sin
    from constants import PI

    samples = []

    for i in range(num_samples):
        u = random.uniform(0.0, 2.0*PI)
        v = random.uniform(0.0, PI)
        x = random.uniform(1.0, band_width_scale) * semi_axis_a * cos(u) * sin(v)
        y = random.uniform(1.0, band_width_scale) * semi_axis_b * sin(u) * sin(v)
        z = random.uniform(1.0, band_width_scale) * semi_axis_c * cos(v)
        samples.append([x, y, z])

    return samples