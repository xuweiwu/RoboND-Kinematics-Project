#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Import modules
import rospy
from tf.transformations import quaternion_matrix
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose

# Use numpy to enhance the performance
from numpy import sqrt, cos, sin, arctan2, array, finfo, float64, pi
from numpy.linalg import inv

# Calculate joint angles using geometric IK method
def inverse_kinematics(pos, R):
    d1 = 0.75
    a1 = 0.35
    a2 = 1.25
    a3 = -0.054
    d4 = 1.50
    d7 = 0.303

    # Calculate positions of the wrist center
    # Calculations differ from lecture notes because the correctional rotation is not yet performed.
    wx = pos[0] - d7 * R[0,0]
    wy = pos[1] - d7 * R[1,0]
    wz = pos[2] - d7 * R[2,0]

    # Calculate intermediate variables
    # Distance between O2 and WC projected on x1-axis
    wx1 = sqrt(wx**2 + wy**2) - a1
    # Distance between O2 and WC projected on z1-axis
    wz1 = wz - d1
    # Distance between O2 and O3
    l2 = a2
    # Distance between O3 and WC
    l3 = sqrt(a3**2 + d4**2)

    # Calculation of theta 1
    q1 = arctan2(wy, wx)
    
    # Calculation of theta 3
    # The cosine law is applied to calculate cos(pi/2 - q3) = sin(q3)
    sq3 = (l2**2 + l3**2 - (wx1**2 + wz1**2))/(2*l2*l3)
    # A positive cq3 implies that q3 is between -pi/2 and pi/2
    # It is a desired range for q3 because uncontinuity will not occur
    cq3 = sqrt(1 - sq3**2)
    # Acoording to the URDF file the limit of joint 3 is [-210, 65]deg    
    # If the resulted q3 is larger than 65 deg, the sign of cq3 needs to be changed
    atanq3 = arctan2(sq3, cq3)
    if atanq3 > 65/180*pi and sq3 < 0:    	
    	q3 = atanq3 - arctan2(-a3, d4) - pi
    else:
    	q3 = atanq3 - arctan2(-a3, d4)
    
    # Calculation of theta 2
    # The cosine law is applied to calculate cos(phi)
    cphi = (wx1**2 + wz1**2 + l2**2 - l3**2)/(2*l2*sqrt(wx1**2 + wz1**2))
    # A positive sphi implies that phi is between 0 to pi 
    sphi = sqrt(1 - cphi**2)    
    # Acoording to the URDF file the limit of joint 2 is [-45, 85]deg
    # The sign of phi should be the same as the sign of arctan2(wx1, wz1)
    # If q3 < -pi/2, then arctan2(wx1, wz1) < 0, the sign of cq3 needs to be changed
    if q3 > -pi/2:
    	q2 = arctan2(wx1, wz1) - arctan2(sphi, cphi)
    else:
    	q2 = arctan2(wx1, wz1) - arctan2(-sphi, cphi)
    
    # Calculate R3_6 through the equation:
    # R0_3 * R3_6 * R_gripper_corr = R_EE => R0_3 = inv(R0_3) * R_EE * R_gripper_corr
    # Both R0_3 and R_gripper_corr_inv are calculated in advance
    R0_3 = array([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
                  [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
                  [        cos(q2 + q3),        -sin(q2 + q3),       0]], dtype=float64)
    R_gripper_corr_inv = array([[0,  0, 1],[0, -1, 0],[1,  0, 0]], dtype=float64)
    R3_6 = inv(R0_3).dot(R).dot(R_gripper_corr_inv)

    # Obtain theta4, theta5, theta6 by solving the inverse orientation problem
    # If wrist singularity occurs, set q4 to zero
    # This part of code is learned from the function euler_from_matrix in transformation.py
    eps = finfo(float).eps * 4.0
    sq5 = sqrt(R3_6[1,0]**2 + R3_6[1,1]**2)
    if sq5 > eps:
        q4 = arctan2(R3_6[2,2], -R3_6[0,2])
        q5 = arctan2(sq5, R3_6[1,2])
        q6 = arctan2(-R3_6[1,1], R3_6[1,0])
    else:
        q4 = arctan2(-R3_6[0,1], R3_6[0,0])
        q5 = 0
        q6 = 0

    return [q1, q2, q3, q4, q5, q6]

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # pos_EE = end-effector position
            # R_EE = end-effector rotation matrix
            # The reference frame of pos_EE and R_EE is the base coordinate system
            pos_EE = [req.poses[x].position.x, req.poses[x].position.y, req.poses[x].position.z]
            R_EE = quaternion_matrix([req.poses[x].orientation.x, req.poses[x].orientation.y, 
                req.poses[x].orientation.z, req.poses[x].orientation.w])[:3,:3]

            # Populate response for the IK request
            joint_trajectory_point.positions = inverse_kinematics(pos_EE, R_EE)
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    	return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
