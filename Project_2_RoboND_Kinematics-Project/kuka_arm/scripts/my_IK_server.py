#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  #Joint angles
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  #link offsets
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  #link lengths
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  #twist angles

	# Create Modified DH parameters
	
	s = {alpha0:0,     a0:0,      d1:0.75,
	     alpha1:-pi/2, a1:0.35,   d2:0,    q2:q2-pi/2,
             alpha2:0,     a2:1.25,   d3:0,
	     alpha3:-pi/2, a3:-0.054, d4:1.5,
	     alpha4:pi/2,  a4:0,      d5:0,
             alpha5:-pi/2, a5:0,      d6:0,
 	     alpha6:0,     a6:0,      d7:0.303,q7:0}

	# Define Modified DH Transformation matrix
	
	def TF_matrix(alpha, a, d, q, s):
	    TF = Matrix([[           cos(q),            sin(q),          0,              a],
                         [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
			 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
			 [               0,                 0,           0,              1]])

  	    return TF.subs(s)
	
	# Create individual transformation matrices
	
	T0_1 = TF_matrix(alpha0, a0, d1, q1, s)
	T1_2 = TF_matrix(alpha1, a1, d2, q2, s)
	T2_3 = TF_matrix(alpha2, a2, d3, q3, s)
	T3_4 = TF_matrix(alpha3, a3, d4, q4, s)
	T4_5 = TF_matrix(alpha4, a4, d5, q5, s)
	T5_6 = TF_matrix(alpha5, a5, d6, q6, s)
	T6_EE = TF_matrix(alpha6, a6, d7, q7, s)

	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

	# Extract rotation matrices from the transformation matrices
	#
	#
        ###


        def rot_x(q):
            r_x = Matrix([[ 1,      0,       0],
                          [ 0, cos(q), -sin(q)],
                          [ 0, sin(q),  cos(q)]])
            return r_x
        
        
        def rot_y(q):
            r_y = Matrix([[  cos(q), 0, sin(q)],
                          [       0, 1,      0],
                          [ -sin(q), 0, cos(q)]])
            return r_y
        
        
        def rot_z(q):
            r_z = Matrix([[ cos(q), -sin(q), 0],
                          [ sin(q),  cos(q), 0],
                          [      0,       0, 1]])
            return r_z

        # Conversion factors between radians and degrees
        rtd = 180 / pi
        dtr = pi / 180
	
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

	    # Step 1: Compute the wrist center

	    # Build EE rotation matrix

	    Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)

            lx = Rrpy[0, 0]
            ly = Rrpy[1, 0]
            lz = Rrpy[2, 0]

            # Calculate the wrist center (test should be (1.85, 0, 1.947)
            wc_x = px - (s[d7] + s[d6]) * lx
            wc_y = py - (s[d7] + s[d6]) * ly
            wc_z = pz - (s[d7] + s[d6]) * lz
            #print('EE location: (%s, %s, %s)' % (wc_x, wc_y, wc_z))

	    # Step 2: Calculate thetas for joint 1, 2 and 3 (determines EE position)

            # Determine the angle for joint 1
            theta1 = atan2(wc_y, wc_x).evalf()
            theta1 = np.clip(theta1, -185*dtr, 185*dtr)
            
            # Find the coordinates of x2, y2, and z2 considering theta 1
            x2 = s[a1] * cos(theta1)
            y2 = s[a1] * sin(theta1)
            z2 = s[d1]

            # Find the x, y, and z distances between joint 2 and wrist center
            X2_WC = wc_x - x2
            Y2_WC = wc_y - y2
            Z2_WC = wc_z - z2

            # Find the distances between joint 2  and the wrist center
            L2_WC = sqrt(X2_WC**2 + Y2_WC**2 + Z2_WC**2)

            # Find the distance between joint 3 and the wrist center
            L3_4 = 0.96     # Distance from joint 3 to joint 4
            L4_5 = 0.54     # Distance from joint 4 to joint 5 (WC)
            L3_4_x = sqrt(L3_4**2 + abs(s[a3])**2)  # x dist from joint 3 to 4
            phi1 = pi - atan2(abs(s[a3]), L3_4_x)
            L3_WC = sqrt(L3_4**2 + L4_5**2 - 2 * L3_4 * L4_5 * cos(phi1))

            # Determine the angle for joint 3
            cos_phi2 = (L2_WC**2 - L3_WC**2 - s[a2]**2) / (-2 * L3_WC * s[a2])
            if abs(cos_phi2) > 1:
                cos_phi2 = 1
                #print('cos_phi2 is greater than 1.')
            phi2 = atan2(sqrt(1 - cos_phi2**2), cos_phi2)
            theta3 = (pi/2 - phi2).evalf()
            theta3 = np.clip(theta3, -210*dtr, (155-90)*dtr)

            # Determine the angle for joint 2
            L2_WC_xy = sqrt(X2_WC**2 + Y2_WC**2)
            phi3 = atan2(Z2_WC, L2_WC_xy)
            cos_phi4 = (L3_WC**2 - L2_WC**2 - s[a2]**2) / (-2 * L2_WC * s[a2])
            if abs(cos_phi4) > 1:
                cos_phi4 = 1
            phi4 = atan2(sqrt(1 - cos_phi4**2), cos_phi4)
            theta2 = (pi/2 - (phi3 + phi4)).evalf()
            theta2 = np.clip(theta2, -45*dtr, 85*dtr)


            # Step 3: Determine rotation matrix for the spherical wrist joints
        
            # Build the transformation matrices of the first 3 joints
            T0_1 = TF_matrix(alpha0, a0, d1, q1, s)
            T1_2 = TF_matrix(alpha1, a1, d2, q2, s)
            T2_3 = TF_matrix(alpha2, a2, d3, q3, s)
    
            # Rotation matrix of the first three joints
            R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={theta1: theta1,
                                                    theta2: theta2,
                                                    theta3: theta3})[0:3, 0:3]

            # Correction to account for orientation difference between
	    #   definition of gripper link in URDF file and the DH convention.
	    #   (rotation around Z axis by 180 deg and X axis by -90 deg)
	    R_corr = simplify(rot_z(pi) * rot_y(-pi/2))
            
            # Calculate the rotation matrix of the spherical wrist joints
            R3_6 = R0_3.T * Rrpy * R_corr

            # Step 4: Calculate the spherical wrist joint angles via Euler angles

            # tf requires a numpy matrix instead of a sympy matrix
            R3_6_np = np.array(R3_6).astype(np.float64)

            # Convert the rotation matrix to Euler angles using tf
            alpha, beta, gamma = tf.transformations.euler_from_matrix(
                R3_6_np, axes='rxyz')   # xyx, yzx, xyz
            theta4 = alpha
            theta5 = beta
            theta6 = gamma

            theta4 = np.pi/2 + theta4
            theta5 = np.pi/2 - theta5

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
