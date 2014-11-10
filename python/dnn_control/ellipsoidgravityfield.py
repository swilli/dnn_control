
import math
import numpy
import sys
import constants

def carlson_elliptic_integral_rd(x, y, z, error_tolerance):
	low_limit = 5.0 * sys.float_info.min
	max_limit = 5.0 * sys.float_info.max
	if not ((min([x,y]) >= 0) and (min([x+y,z]) >= low_limit) and (max([x,y,z]) < max_limit)):
		return numpy.nan

	xn = x
	yn = y
	zn = z
	sigma = 0.0
	power4 = 1.0

	mu = (xn + yn + 3.0*zn)*0.2
	xn_dev = (mu - xn)/mu
	yn_dev = (mu - yn)/mu
	zn_dev = (mu - zn)/mu
	epsilon = max([math.fabs(val) for val in [xn_dev,yn_dev,zn_dev]])

	while epsilon >= error_tolerance:
		xn_root = math.sqrt(xn)
		yn_root = math.sqrt(yn)
		zn_root = math.sqrt(zn)
		val_lambda = xn_root*(yn_root + zn_root) + yn_root*zn_root
		sigma = sigma + power4/(zn_root*(zn + val_lambda))
		power4 = 0.25*power4
		xn = 0.25*(xn + val_lambda)
		yn = 0.25*(yn + val_lambda)
		zn = 0.25*(zn + val_lambda)

		mu = (xn + yn + 3.0*zn)*0.2
		xn_dev = (mu - xn)/mu
		yn_dev = (mu - yn)/mu
		zn_dev = (mu - zn)/mu
		epsilon = max([math.fabs(val) for val in [xn_dev,yn_dev,zn_dev]])

	coef_1 = 3.0/14.0
	coef_2 = 1.0/6.0
	coef_3 = 9.0/22.0
	coef_4 = 3.0/26.0
	ea = xn_dev*yn_dev
	eb = zn_dev**2
	ec = ea - eb
	ed = ea - 6.0*eb
	ef = ed + 2.0*ec
	s1 = ed*(-coef_1 + 0.25*coef_3*ed - 1.5*coef_4*zn_dev*ef)
	s2 = zn_dev*(coef_2*ef + zn_dev*(-coef_3*ec + zn_dev*coef_4*ea))
	return 3.0*sigma + power4*(1.0 + s1 + s2)/(mu*math.sqrt(mu))

def carlson_elliptic_integral_rf(x, y, z, error_tolerance):
	low_limit = 5.0 * sys.float_info.min
	max_limit = 5.0 * sys.float_info.max

	if not ((min([x,y,z]) >= 0) and (min([x+y,x+z,y+z]) >= low_limit) and (max([x,y,z]) < max_limit)):
		return numpy.nan

	xn = x
	yn = y
	zn = z
	mu = (xn + yn + zn)/3.0
	xn_dev = 2.0 - (mu + xn)/mu
	yn_dev = 2.0 - (mu + yn)/mu
	zn_dev = 2.0 - (mu + zn)/mu
	epsilon = max([math.fabs(val) for val in [xn_dev,yn_dev,zn_dev]])

	while epsilon >= error_tolerance:
		xn_root = math.sqrt(xn)
		yn_root = math.sqrt(yn)
		zn_root = math.sqrt(zn)
		val_lambda = xn_root*(yn_root + zn_root) + yn_root*zn_root
		xn = 0.25*(xn + val_lambda)
		yn = 0.25*(yn + val_lambda)
		zn = 0.25*(zn + val_lambda)

		mu = (xn + yn + zn)/3.0
		xn_dev = 2.0 - (mu + xn)/mu
		yn_dev = 2.0 - (mu + yn)/mu
		zn_dev = 2.0 - (mu + zn)/mu
		epsilon = max([math.fabs(val) for val in [xn_dev,yn_dev,zn_dev]])

	coef_1 = 1.0/24.0
	coef_2 = 3.0/44.0
	coef_3 = 1.0/14.0
	e2 = xn_dev*yn_dev - zn_dev**2
	e3 = xn_dev*yn_dev*zn_dev
	s = 1.0 + (coef_1*e2 - 0.1 - coef_2*e3)*e2 + coef_3*e3
	return s/math.sqrt(mu)
	
def legendre_elliptic_integral_pf(phi, k, error_tolerance):
	sin_phi = math.sin(phi)
	cos_phi = math.cos(phi)
	cos_phi_pow2 = cos_phi**2
	sin_phi_pow2 = sin_phi**2
	k_pow2 = k**2
	y = 1.0 - k_pow2 * sin_phi_pow2
	return sin_phi*carlson_elliptic_integral_rf(cos_phi_pow2, y, 1.0, error_tolerance)

def legendre_elliptic_integral_pe(phi, k, error_tolerance):
	sin_phi = math.sin(phi)
	cos_phi = math.cos(phi)
	cos_phi_pow2 = cos_phi**2
	sin_phi_pow2 = sin_phi**2
	k_pow2 = k**2
	y = 1.0 - k_pow2 * sin_phi_pow2
	return sin_phi*carlson_elliptic_integral_rf(cos_phi_pow2, y, 1.0, error_tolerance) - k_pow2*sin_phi*sin_phi_pow2*carlson_elliptic_integral_rd(cos_phi_pow2, y, 1.0, error_tolerance)/3.0


def ellipsoid_gravity_field(position ,semi_axis_a, semi_axis_b, semi_axis_c, density):
	acceleration = [0, 0, 0]
	error_tolerance = 1e-10
	
	semi_axis_a_pow2 = semi_axis_a**2
	semi_axis_b_pow2 = semi_axis_b**2
	semi_axis_c_pow2 = semi_axis_c**2

	pos_z = position[2]
 	pos_z_pow_2 = pos_z**2
 	pos_x_pow_2 = position[0]**2
 	pos_y_pow_2 = position[1]**2

	coef_2 = -(pos_x_pow_2 + pos_y_pow_2 + pos_z_pow_2 - semi_axis_a_pow2 - semi_axis_b_pow2 - semi_axis_c_pow2)
	coef_1 = -semi_axis_c_pow2*(pos_x_pow_2 + pos_y_pow_2) + semi_axis_b_pow2*(semi_axis_c_pow2 - pos_x_pow_2 - pos_z_pow_2) + semi_axis_a_pow2*(semi_axis_b_pow2 + semi_axis_c_pow2 - pos_y_pow_2 - pos_z_pow_2)
	coef_0 = -semi_axis_b_pow2*semi_axis_c_pow2*pos_x_pow_2 + semi_axis_a_pow2*(-semi_axis_c_pow2*pos_y_pow_2 + semi_axis_b_pow2*(semi_axis_c - pos_z)*(semi_axis_c + pos_z))
	polynomial = numpy.array([1.0, coef_2, coef_1, coef_0])
	maximum_root = max(numpy.roots(polynomial))

	phi = math.asin(math.sqrt((semi_axis_a_pow2 - semi_axis_c_pow2)/(maximum_root + semi_axis_a_pow2)))
	k = math.sqrt((semi_axis_a_pow2 - semi_axis_b_pow2)/(semi_axis_a_pow2 - semi_axis_c_pow2))

	integral_f1 = legendre_elliptic_integral_pf(phi, k, error_tolerance)
	integral_e1 = legendre_elliptic_integral_pe(phi, k, error_tolerance)

	fac_1 = 4.0*constants.PI*constants.GRAVITATIONAL_CONSTANT*density*semi_axis_a*semi_axis_b*semi_axis_c/math.sqrt(semi_axis_a_pow2 - semi_axis_c_pow2)
	fac_2 = math.sqrt((semi_axis_a_pow2 - semi_axis_c_pow2)/((semi_axis_a_pow2 + maximum_root)*(semi_axis_b_pow2 + maximum_root)*(semi_axis_c_pow2 + maximum_root)))

	acceleration[0] = fac_1/(semi_axis_a_pow2 - semi_axis_b_pow2)*(integral_e1 - integral_f1)
	acceleration[1] = fac_1*((semi_axis_c_pow2 - semi_axis_a_pow2)*integral_e1/((semi_axis_a_pow2 - semi_axis_b_pow2)*(semi_axis_b_pow2 - semi_axis_c_pow2)) + integral_f1/(semi_axis_a_pow2 - semi_axis_b_pow2) + (semi_axis_c_pow2 + maximum_root)*fac_2/(semi_axis_b_pow2 - semi_axis_c_pow2))
	acceleration[2] = fac_1*((semi_axis_b_pow2 + maximum_root)*fac_2 - integral_e1)/(semi_axis_c_pow2 - semi_axis_b_pow2)
	return acceleration