#!/usr/bin/env python

# This file is for verifying the calculations of forward kinematics

# import modules
import rospy
import numpy as np
from tf.transformations import euler_from_matrix, quaternion_from_euler, translation_from_matrix
from sensor_msgs.msg import JointState
from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
from sympy.utilities.autowrap import autowrap


# Define FKCalculation class to do the major symbolic calculation by initialization
class FKCalculation():
	def __init__(self):
		# Create symbols for joint variables
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
		theta = [q1, q2, q3, q4, q5, q6]
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

		# DH parameters
		s = {alpha0:     0,  a0:      0,  d1:  0.75, 
			 alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2-pi/2,
			 alpha2:     0,  a2:   1.25,  d3:     0,
			 alpha3: -pi/2,  a3: -0.054,  d4:  1.50,
			 alpha4:  pi/2,  a4:      0,  d5:     0,  
			 alpha5: -pi/2,  a5:      0,  d6:     0,
			 alpha6:     0,  a6:      0,  d7: 0.303,  q7:       0}

		# Individual homogeneous transforms
		T0_1 = Matrix([[            cos(q1), 		    -sin(q1),            0,              a0],
					   [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
					   [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
					   [                  0,                   0,            0,               1]])
		T0_1 = T0_1.subs(s)
		T1_2 = Matrix([[            cos(q2), 		    -sin(q2),            0,              a1],
					   [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
					   [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
					   [                  0,                   0,            0,               1]])
		T1_2 = T1_2.subs(s)
		T2_3 = Matrix([[            cos(q3), 		    -sin(q3),            0,              a2],
					   [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
					   [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
					   [                  0,                   0,            0,               1]])
		T2_3 = T2_3.subs(s)    	              	               
		T3_4 = Matrix([[            cos(q4), 		    -sin(q4),            0,              a3],
					   [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
					   [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
					   [                  0,                   0,            0,               1]])
		T3_4 = T3_4.subs(s)
		T4_5 = Matrix([[            cos(q5), 		    -sin(q5),            0,              a4],
					   [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
					   [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
					   [                  0,                   0,            0,               1]])
		T4_5 = T4_5.subs(s)
		T5_6 = Matrix([[            cos(q6), 		    -sin(q6),            0,              a5],
					   [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
					   [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
					   [                  0,                   0,            0,               1]])
		T5_6 = T5_6.subs(s)
		T6_G = Matrix([[            cos(q7), 		    -sin(q7),            0,              a6],
					   [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
					   [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
					   [                  0,                   0,            0,               1]])
		T6_G = T6_G.subs(s)

		# Correction needed to cccount of orientation difference between definition of
		# gripper_link in URDF versus DH convention
		R_z = Matrix([[             cos(pi),    	    -sin(pi),            0,               0],
					  [             sin(pi),             cos(pi),            0,               0],
					  [                   0,                   0,            1,               0],
					  [                   0,                   0,            0,               1]])
		R_y = Matrix([[          cos(-pi/2), 		           0,   sin(-pi/2),               0],
					  [                   0,                   1,            0,               0],
					  [         -sin(-pi/2),                   0,   cos(-pi/2),               0],
					  [                   0,                   0,            0,               1]])
		T_gripper_corr = R_z * R_y

		# Correction needed to account of position difference between definition of
		# joint_1, joint_4, and joint_6 in URDF versus DH convention
		T_1_corr = Matrix([[              1,    	           0,            0,               0],
						   [              0,                   1,            0,               0],
						   [              0,                   0,            1,           -0.42],
						   [              0,                   0,            0,               1]])
		T_4_corr = Matrix([[              1,    	           0,            0,               0],
						   [              0,                   1,            0,               0],
						   [              0,                   0,            1,           -0.54],
						   [              0,                   0,            0,               1]])
		T_6_corr = Matrix([[              1,    	           0,            0,               0],
						   [              0,                   1,            0,               0],
						   [              0,                   0,            1,            0.193],
						   [              0,                   0,            0,               1]])

		# Composition of Homogeneous Transforms    	
		T0_2 = simplify(T0_1 * T1_2)
		T0_3 = simplify(T0_2 * T2_3)
		T0_4 = simplify(T0_3 * T3_4)
		T0_5 = simplify(T0_4 * T4_5)
		T0_6 = simplify(T0_5 * T5_6)
		T0_G = simplify(T0_6 * T6_G)

		# Apply correction transforms
		T0_1 = simplify(T0_1 * T_1_corr)
		T0_4 = simplify(T0_4 * T_4_corr)
		T0_6 = simplify(T0_6 * T_6_corr)
		T0_G = simplify(T0_G * T_gripper_corr)
		
		# Use autowrap to compile and wrap a binary python function
		# The compiled binary can achieve better performance than SymPy’s .evalf()
		self.T0_1 = autowrap(T0_1, args=theta)
		self.T0_2 = autowrap(T0_2, args=theta)
		self.T0_3 = autowrap(T0_3, args=theta)
		self.T0_4 = autowrap(T0_4, args=theta)
		self.T0_5 = autowrap(T0_5, args=theta)
		self.T0_6 = autowrap(T0_6, args=theta)
		self.T0_G = autowrap(T0_G, args=theta)

# Initialize fk_cal
fk_cal = FKCalculation()
 
def handle_calculate_FK(data):

	# Extract joint positions
	# The first two positions corresponde to right/left gripper finger joints and will not be used
	joint_states = data.position[2:]

	# Calculate tanslation and rotation of target link with respect to base_link
	# Here target link set to gripper_link
	T0_G = np.array(fk_cal.T0_G(joint_states[0], joint_states[1], joint_states[2], \
								joint_states[3], joint_states[4], joint_states[5]))
	(roll, pitch, yaw) = euler_from_matrix(T0_G, 'sxyz')
	(x, y, z, w) = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
	(px, py, pz) = translation_from_matrix(T0_G)

	# Print out tanslation and rotation of target link with respect to base_link
	print "- Translation: [%.3f, %.3f, %.3f]" % (px, py, pz)
	print "- Rotation: in Quaternion   [%.3f, %.3f, %.3f, %.3f]" % (x, y, z, w)
	print "            in RPY (radian) [%.3f, %.3f, %.3f]" % (roll, pitch, yaw)
	print "            in RPY (degree) [%.3f, %.3f, %.3f]" % (roll*180.0/np.pi, pitch*180.0/np.pi, yaw*180.0/np.pi)

def FK_test():
	# Create a joint_states listener
	rospy.init_node('FK_test')
	rospy.Subscriber("joint_states", JointState, handle_calculate_FK)
	rospy.spin()

if __name__ == "__main__":
	FK_test()