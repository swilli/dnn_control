from sys import *
from math import *
from numpy import *
from ellipsoidgravityfield import *
from constants import *

class Asteroid():

	def __init__(self, semi_axis_a, semi_axis_b, semi_axis_c, density):

		# for now, only allow semi_axis_b == semi_axis_c 
		#if abs(semi_axis_b -semi_axis_c) > 1e-10:
		#	print("Asteroid semi axis b and c have to be of same length.")
		#	exit()

		self.density = float(density)
		self.semi_axis_a = float(semi_axis_a)
		self.semi_axis_a_pow2 = self.semi_axis_a**2
		self.semi_axis_b = float(semi_axis_b)
		self.semi_axis_b_pow2 = self.semi_axis_b**2
		self.semi_axis_c = float(semi_axis_c)
		self.semi_axis_c_pow2 = self.semi_axis_c**2


	# Ported from "Ellipsoid_Gravity_Field.m" written by Dario Cersosimo, October 16, 2009.
	def gravity_at_position(self, position):
		acceleration = [0, 0, 0]
		error_tolerance = 1e-10
	
		density = self.density
		semi_axis_a = self.semi_axis_a
		semi_axis_b = self.semi_axis_b
		semi_axis_c = self.semi_axis_c
		semi_axis_a_pow2 = self.semi_axis_a_pow2
		semi_axis_b_pow2 = self.semi_axis_b_pow2
		semi_axis_c_pow2 = self.semi_axis_c_pow2

		pos_z = position[2]
 		pos_z_pow_2 = pos_z**2
 		pos_x_pow_2 = position[0]**2
 		pos_y_pow_2 = position[1]**2

		coef_2 = -(pos_x_pow_2 + pos_y_pow_2 + pos_z_pow_2 - semi_axis_a_pow2 - semi_axis_b_pow2 - semi_axis_c_pow2)
		coef_1 = -semi_axis_c_pow2*(pos_x_pow_2 + pos_y_pow_2) + semi_axis_b_pow2*(semi_axis_c_pow2 - pos_x_pow_2 - pos_z_pow_2) + semi_axis_a_pow2*(semi_axis_b_pow2 + semi_axis_c_pow2 - pos_y_pow_2 - pos_z_pow_2)
		coef_0 = -semi_axis_b_pow2*semi_axis_c_pow2*pos_x_pow_2 + semi_axis_a_pow2*(-semi_axis_c_pow2*pos_y_pow_2 + semi_axis_b_pow2*(semi_axis_c - pos_z)*(semi_axis_c + pos_z))
		polynomial = array([1.0, coef_2, coef_1, coef_0])
		maximum_root = max(roots(polynomial))

		phi = asin(sqrt((semi_axis_a_pow2 - semi_axis_c_pow2)/(maximum_root + semi_axis_a_pow2)))
		k = sqrt((semi_axis_a_pow2 - semi_axis_b_pow2)/(semi_axis_a_pow2 - semi_axis_c_pow2))

		integral_f1 = legendre_elliptic_integral_pf(phi, k, error_tolerance)
		integral_e1 = legendre_elliptic_integral_pe(phi, k, error_tolerance)

		fac_1 = 4.0*PI*GRAVITATIONAL_CONSTANT*density*semi_axis_a*semi_axis_b*semi_axis_c/sqrt(semi_axis_a_pow2 - semi_axis_c_pow2)
		fac_2 = sqrt((semi_axis_a_pow2 - semi_axis_c_pow2)/((semi_axis_a_pow2 + maximum_root)*(semi_axis_b_pow2 + maximum_root)*(semi_axis_c_pow2 + maximum_root)))

		acceleration[0] = fac_1/(semi_axis_a_pow2 - semi_axis_b_pow2)*(integral_e1 - integral_f1)
		acceleration[1] = fac_1*((semi_axis_c_pow2 - semi_axis_a_pow2)*integral_e1/((semi_axis_a_pow2 - semi_axis_b_pow2)*(semi_axis_b_pow2 - semi_axis_c_pow2)) + integral_f1/(semi_axis_a_pow2 - semi_axis_b_pow2) + (semi_axis_c_pow2 + maximum_root)*fac_2/(semi_axis_b_pow2 - semi_axis_c_pow2))
		acceleration[2] = fac_1*((semi_axis_b_pow2 + maximum_root)*fac_2 - integral_e1)/(semi_axis_c_pow2 - semi_axis_b_pow2)
		return acceleration