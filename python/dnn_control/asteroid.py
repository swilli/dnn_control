class Asteroid:

    """
    This class represents the dynamics and gravity of an asteroid shaped as a rigid ellipsoid having different
    rotational properties along its three principal inertia axis (Ix, Iy, Iz).

    For the angular velocity dynamics the analytical solution in terms of Jacobi elliptic functions is used (Implementation
    largely inspired from Landau Lifshitz Mechanics paragrpah 37). We thus assume w_y(-t_bias) = 0. So we only are free to specify two
    more initial conditions and we do so by assigning kinetic energy and angular momentum. As a result the user is not free to specify
    the initial angular velocity, but only the values of the two prime integrals. He does so by defining an angular velocity vector in
    the constructor, this is used only to compute kinetic energy and rotational momentum, the second of its components (w_y) will thus be disregarded
    as 0 will instead be used.

    For the asteroid gravity, we use the implementation of Dario Cersosimo whi kindly sent us his matlab files.
    """

    def __init__(self, semi_axis, density, angular_velocity, time_bias):

        """
        This constructs the asteroid object from its inertia properties and its prime integrals value

        USAGE: ast = Asteroid(0.1,0.2,0.3,2.3,[0.23,0.1,0.56],234)

        * semi_axis:        Semi Axis a,b,c  where a < b < c (UNITS)
        * density:                                      Asteroid density (UNITS)
        * angular_velocity:                             Asteroid hypothetical angular velocity (used to define prime integrals only)
        * time_bias:                                    A time bias applied so that w_y(-t_bias) = 0

        """

        from math import sqrt
        from constants import PI, GRAVITATIONAL_CONSTANT

        # SU controls (Stupid User)
        if (semi_axis[0] == semi_axis[1]) or (semi_axis[0] == semi_axis[2]) or (semi_axis[1] == semi_axis[2]):
            raise Exception("This function does not work for symmetric tops, make sure Ix, Iy and Iz are different")
        if sorted(semi_axis[:]) != semi_axis[::-1]:
            raise Exception("This function assumes semi axis c < b < a, please make sure this is the case")
        if angular_velocity[0] == 0 or angular_velocity[2] == 0:
            raise Exception("You cannot define zero values for w_x or w_z as this "
                            "functions already assumes w_y=0 (thus the motion would be a rotation")

        angular_velocity = [float(val) for val in angular_velocity]

        self._semi_axis = [float(val) for val in semi_axis]
        self._semi_axis_pow2 = [val * val for val in self._semi_axis]
        self._density = float(density)
        self._time_bias = float(time_bias)
        self._mass = 4.0 / 3.0 * PI * self._semi_axis[0] * self._semi_axis[1] * self._semi_axis[2] * self._density

        self._inertia = [0.0, 0.0, 0.0]
        self._inertia[0] = 0.2 * self._mass * (self._semi_axis_pow2[1] + self._semi_axis_pow2[2])
        self._inertia[1] = 0.2 * self._mass * (self._semi_axis_pow2[0] + self._semi_axis_pow2[2])
        self._inertia[2] = 0.2 * self._mass * (self._semi_axis_pow2[0] + self._semi_axis_pow2[1])
        self._inertia_pow2 = [val * val for val in self._inertia]

        signs = [0.0, 0.0, 0.0]
        signs[0] = (-1.0, 1.0)[angular_velocity[0] > 0]
        signs[1] = (-1.0, 1.0)[angular_velocity[0] * angular_velocity[2] > 0]
        signs[2] = (-1.0, 1.0)[angular_velocity[2] > 0]

        self._energy_mul2 = sum([val_i * val_omega ** 2 for val_i, val_omega in zip(self._inertia, angular_velocity)])
        self._momentum_pow2 = sum([val_i ** 2 * val_omega ** 2 for val_i, val_omega in zip(self._inertia, angular_velocity)])

        inertia = self._inertia[:]

        self._inversion = False
        if self._momentum_pow2 < self._energy_mul2 * self._inertia[1]:
            self._inversion = True
            inertia[2] = self._inertia[0]
            inertia[0] = self._inertia[2]
            signs[0], signs[2] = signs[2], signs[0]


        # Lifshitz eq (37.10)
        self._elliptic_coefficients = [0.0, 0.0, 0.0]

        self._elliptic_coefficients[0] = signs[0] * sqrt((self._energy_mul2 * inertia[2] - self._momentum_pow2)
                                                         / (inertia[0] * (inertia[2] - inertia[0])))
        self._elliptic_coefficients[1] = signs[1] * sqrt((self._energy_mul2 * inertia[2] - self._momentum_pow2)
                                                         / (inertia[1] * (inertia[2] - inertia[1])))
        self._elliptic_coefficients[2] = signs[2] * sqrt((self._momentum_pow2 - self._energy_mul2 * inertia[0])
                                                         / (inertia[2] * (inertia[2] - inertia[0])))

        # Lifshitz eq (37.8)
        self._elliptic_tau = sqrt((inertia[2] - inertia[1]) * (self._momentum_pow2 - self._energy_mul2 * inertia[0])
                                  / (inertia[0] * inertia[1] * inertia[2]))

        # Lifshitz eq (37.9)
        self._elliptic_modulus = (inertia[1] - inertia[0]) * (self._energy_mul2 * inertia[2] - self._momentum_pow2) \
                                 / ((inertia[2] - inertia[1]) * (self._momentum_pow2 - self._energy_mul2 * inertia[0]))

        # Cersosimo eq (3.12)
        self._gamma = 4.0 * PI * GRAVITATIONAL_CONSTANT * self._density\
                      * self._semi_axis[0] * self._semi_axis[1] * self._semi_axis[2] \
                      / sqrt(self._semi_axis_pow2[0] - self._semi_axis_pow2[2])

    def get_semi_axis(self, dimension):
        return self._semi_axis[dimension]

    # Implementation of eq. (3.11a-c) from "EVALUATION OF NOVEL HOVERING STRATEGIES
    # TO IMPROVE GRAVITY-TRACTOR DEFLECTION MERITS" written by Dario Cersosimo,
    def gravity_at_position(self, position):
        from math import asin, sqrt
        from numpy import array, roots
        from scipy.special import ellipkinc, ellipeinc
        from sys import float_info

        gravity = [0.0, 0.0, 0.0]


        pos_x_pow2 = position[0] * position[0]
        pos_y_pow2 = position[1] * position[1]
        pos_z_pow2 = position[2] * position[2]

        # Cersosimo eq (3.7)
        coef_3 = 1.0
        coef_2 = -(pos_x_pow2 + pos_y_pow2 + pos_z_pow2
                   - self._semi_axis_pow2[0] - self._semi_axis_pow2[1] - self._semi_axis_pow2[2])
        coef_1 = -(self._semi_axis_pow2[1] * pos_x_pow2 + self._semi_axis_pow2[2] * pos_x_pow2
                   + self._semi_axis_pow2[0] * pos_y_pow2 + self._semi_axis_pow2[2] * pos_y_pow2
                   + self._semi_axis_pow2[0] * pos_z_pow2 + self._semi_axis_pow2[1] * pos_z_pow2
                   - self._semi_axis_pow2[0] * self._semi_axis_pow2[2]
                   - self._semi_axis_pow2[1] * self._semi_axis_pow2[2]
                   - self._semi_axis_pow2[0] * self._semi_axis_pow2[1])
        coef_0 = -(self._semi_axis_pow2[1] * self._semi_axis_pow2[2] * pos_x_pow2
                   + self._semi_axis_pow2[0] * self._semi_axis_pow2[2] * pos_y_pow2
                   + self._semi_axis_pow2[0] * self._semi_axis_pow2[1] * pos_z_pow2
                   - self._semi_axis_pow2[0] * self._semi_axis_pow2[1] * self._semi_axis_pow2[2])

        poly_coefs = array([coef_3, coef_2, coef_1, coef_0])
        kappa = float_info.min
        for root in roots(poly_coefs):
            if root > kappa:
                kappa = root

        phi = asin(sqrt((self._semi_axis_pow2[0] - self._semi_axis_pow2[2]) / (kappa + self._semi_axis_pow2[0])))
        k = (self._semi_axis_pow2[0] - self._semi_axis_pow2[1]) / (self._semi_axis_pow2[0] - self._semi_axis_pow2[2])

        integral_F = ellipkinc(phi, k)
        integral_E = ellipeinc(phi, k)

        # Cersosimo eq (3.13)
        delta = sqrt((self._semi_axis_pow2[0] - self._semi_axis_pow2[2])
                     / ((self._semi_axis_pow2[0] + kappa)
                        * (self._semi_axis_pow2[1] + kappa)
                        * (self._semi_axis_pow2[2] + kappa)))

        gravity[0] = self._gamma / (self._semi_axis_pow2[0] - self._semi_axis_pow2[1]) * (integral_E - integral_F)
        gravity[1] = self._gamma * ((-self._semi_axis_pow2[0] + self._semi_axis_pow2[2]) * integral_E
                                   / ((self._semi_axis_pow2[0] - self._semi_axis_pow2[1])
                                      * (self._semi_axis_pow2[1] - self._semi_axis_pow2[2]))
                                   + integral_F / (self._semi_axis_pow2[0] - self._semi_axis_pow2[1])
                                   + delta * (self._semi_axis_pow2[2] + kappa) / (self._semi_axis_pow2[1] - self._semi_axis_pow2[2]))

        gravity[2] = self._gamma / (self._semi_axis_pow2[2] - self._semi_axis_pow2[1]) \
                     * (-integral_E + (self._semi_axis_pow2[1] + kappa) * delta)

        gravity[0] *= position[0]
        gravity[1] *= position[1]
        gravity[2] *= position[2]

        return gravity

    # compute w
    def angular_velocity_and_acceleration_at_time(self, time):
        from scipy.special import ellipj

        # Lifshitz eq (37.8)
        t = (time + self._time_bias) * self._elliptic_tau

        # Get analytical solution
        sn_tau, cn_tau, dn_tau, _ = ellipj(t, self._elliptic_modulus)

        # Lifshitz eq (37.10)
        if self._inversion:
            angular_velocity = [self._elliptic_coefficients[2] * dn_tau,
                                self._elliptic_coefficients[1] * sn_tau,
                                self._elliptic_coefficients[0] * cn_tau]
        else:
            angular_velocity = [self._elliptic_coefficients[0] * cn_tau,
                                self._elliptic_coefficients[1] * sn_tau,
                                self._elliptic_coefficients[2] * dn_tau]

        # Lifshitz eq (36.5)
        angular_acceleration = [(self._inertia[1] - self._inertia[2]) * angular_velocity[1] * angular_velocity[2]
                                / self._inertia[0],
                                (self._inertia[2] - self._inertia[0]) * angular_velocity[2] * angular_velocity[0]
                                / self._inertia[1],
                                (self._inertia[0] - self._inertia[1]) * angular_velocity[0] * angular_velocity[1]
                                / self._inertia[2]]

        return angular_velocity, angular_acceleration

    def _nearest_point_on_ellipse_first_quadrant(self, semi_axis, position):
        from math import sqrt
        from scipy.optimize import bisect

        solution = [0.0, 0.0]

        semi_axis_pow2 = [semi_axis[0] * semi_axis[0], semi_axis[1] * semi_axis[1]]

        if position[1] > 0.0:
            if position[0] > 0.0:
                semi_axis_mul_pos = [semi_axis[0] * position[0], semi_axis[1] * position[1]]
                lower_boundary = 0.0
                upper_boundary = sqrt(semi_axis_mul_pos[0] * semi_axis_mul_pos[0]
                                      + semi_axis_mul_pos[1] * semi_axis_mul_pos[1])

                def fun(time):
                    cur_position = [semi_axis_mul_pos[0] / (time + semi_axis_pow2[0]),
                                    semi_axis_mul_pos[1] / (time + semi_axis_pow2[1])]
                    return cur_position[0] * cur_position[0] + cur_position[1] * cur_position[1] - 1.0

                time = bisect(fun, lower_boundary, upper_boundary)
                solution = [semi_axis_pow2[0] * position[0] / (time + semi_axis_pow2[0]),
                            semi_axis_pow2[1] * position[1] / (time + semi_axis_pow2[1])]
            else:
                solution[0] = 0.0
                solution[1] = semi_axis[1]
        else:
            denominator = semi_axis_pow2[0] - semi_axis_pow2[1]
            semi_axis_mul_pos = semi_axis[0] * position[0]
            if semi_axis_mul_pos < denominator:
                semi_axis_div_denom = semi_axis_mul_pos / denominator
                semi_axis_div_denom_pow2 = semi_axis_div_denom * semi_axis_div_denom
                solution[0] = semi_axis[0] * semi_axis_div_denom
                solution[1] = semi_axis[1] * sqrt(abs(1.0 - semi_axis_div_denom_pow2))
            else:
                solution[0] = semi_axis[0]
                solution[1] = 0.0

        distance = sqrt((position[0] - solution[0]) * (position[0] - solution[0])
                        + (position[1] - solution[1]) * (position[1] - solution[1]))

        return solution, distance

    def _nearest_point_on_ellipsoid_at_position_first_quadrant(self, position):
        from math import sqrt
        from scipy.optimize import bisect

        solution = [0.0, 0.0, 0.0]

        if position[2] > 0.0:
            if position[1] > 0.0:
                if position[0] > 0.0:
                    semi_axis_mul_pos = [self._semi_axis[0] * position[0],
                                         self._semi_axis[1] * position[1],
                                         self._semi_axis[2] * position[2]]
                    lower_boundary = 0.0
                    upper_boundary = sqrt(semi_axis_mul_pos[0] * semi_axis_mul_pos[0]
                                          + semi_axis_mul_pos[1] * semi_axis_mul_pos[1]
                                          + semi_axis_mul_pos[2] * semi_axis_mul_pos[2])

                    def fun(time):
                        cur_position = [semi_axis_mul_pos[0] / (time + self._semi_axis_pow2[0]),
                                        semi_axis_mul_pos[1] / (time + self._semi_axis_pow2[1]),
                                        semi_axis_mul_pos[2] / (time + self._semi_axis_pow2[2])]
                        return cur_position[0] * cur_position[0] + cur_position[1] * cur_position[1] \
                               + cur_position[2] * cur_position[2] - 1.0

                    time = bisect(fun, lower_boundary, upper_boundary)
                    solution = [self._semi_axis_pow2[0] * position[0] / (time + self._semi_axis_pow2[0]),
                                self._semi_axis_pow2[1] * position[1] / (time + self._semi_axis_pow2[1]),
                                self._semi_axis_pow2[2] * position[2] / (time + self._semi_axis_pow2[2])]
                else:
                    solution[0] = 0.0
                    semi_axis_2d = self._semi_axis[1:3]
                    position_2d = position[1:3]
                    solution_2d, _ = self._nearest_point_on_ellipse_first_quadrant(semi_axis_2d, position_2d)
                    solution[1] = solution_2d[0]
                    solution[2] = solution_2d[1]
            else:
                solution[1] = 0.0
                if position[0] > 0.0:
                    semi_axis_2d = [self._semi_axis[0], self._semi_axis[2]]
                    position_2d = [position[0], position[2]]
                    solution_2d, _ = self._nearest_point_on_ellipse_first_quadrant(semi_axis_2d, position_2d)
                    solution[0] = solution_2d[0]
                    solution[2] = solution_2d[1]
                else:
                    solution[0] = 0.0
                    solution[2] = self._semi_axis[2]
        else:
            denominator = [self._semi_axis_pow2[0] - self._semi_axis_pow2[2],
                           self._semi_axis_pow2[1] - self._semi_axis_pow2[2]]
            semi_axis_mul_pos = [self._semi_axis[0] * position[0], self._semi_axis[1] * position[1]]
            if semi_axis_mul_pos[0] < denominator[0] and semi_axis_mul_pos[1] < denominator[1]:
                semi_axis_div_denom = [semi_axis_mul_pos[0] / denominator[0],
                                       semi_axis_mul_pos[1] / denominator[1]]
                semi_axis_div_denom_pow2 = [semi_axis_div_denom[0] * semi_axis_div_denom[0],
                                            semi_axis_div_denom[1] * semi_axis_div_denom[1]]
                discr = 1.0 - semi_axis_div_denom_pow2[0] - semi_axis_div_denom_pow2[1]
                if discr > 0.0:
                    solution[0] = self._semi_axis[0] * semi_axis_div_denom[0]
                    solution[1] = self._semi_axis[1] * semi_axis_div_denom[1]
                    solution[2] = self._semi_axis[2] * sqrt(discr)
                else:
                    solution[2] = 0.0
                    semi_axis_2d = self._semi_axis[0:2]
                    position_2d = position[0:2]
                    solution_2d, _ = self._nearest_point_on_ellipse_first_quadrant(semi_axis_2d, position_2d)
                    solution[0] = solution_2d[0]
                    solution[1] = solution_2d[1]
            else:
                solution[2] = 0.0
                semi_axis_2d = self._semi_axis[0:2]
                position_2d = position[0:2]
                solution_2d, _ = self._nearest_point_on_ellipse_first_quadrant(semi_axis_2d, position_2d)
                solution[0] = solution_2d[0]
                solution[1] = solution_2d[1]

        distance = sqrt((position[0] - solution[0]) * (position[0] - solution[0])
                        + (position[1] - solution[1]) * (position[1] - solution[1])
                        + (position[2] - solution[2]) * (position[2] - solution[2]))

        return solution, distance

    def nearest_point_on_surface_to_position(self, position):
        '''
            Port of Implementation from "Distance from a Point to an Ellipse, an Ellipsoid, or a Hyperellipsoid" by David Eberly, 2013.
        '''
        from math import copysign

        signs = [copysign(1.0, val) for val in position]

        abs_position = [abs(var) for var in position]

        surface_position, distance = self._nearest_point_on_ellipsoid_at_position_first_quadrant(abs_position)

        surface_position = [signs[0] * surface_position[0],
                            signs[1] * surface_position[1],
                            signs[2] * surface_position[2]]

        return surface_position, distance