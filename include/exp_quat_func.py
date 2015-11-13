#!/usr/bin/env python
"""Exponential and Quaternion code for Lab 6.
Course: EE 106, Fall 2015
Author: Victor Shia, 9/24/15

This Python file is a code skeleton Lab 6 which calculates the rigid body transform
given a rotation / translation and computes the twist from rigid body transform.

When you think you have the methods implemented correctly, you can test your 
code by running "python exp_quat_func.py at the command line.

This code requires the NumPy and SciPy libraries and kin_func_skeleton which you 
should have written in lab 3. If you don't already have 
these installed on your personal computer, you can use the lab machines or 
the Ubuntu+ROS VM on the course page to complete this portion of the homework.
"""

import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import robot206

def quaternion_to_exp(rot):
    """
    Converts a quaternion vector in 3D to its corresponding omega and theta.
    This uses the quaternion -> exponential coordinate equation given in Lab 6
    
    Args:
    rot - a (4,) nd array or 4x1 array: the quaternion vector (\vec{q}, q_o)
    
    Returns:
    omega - (3,) ndarray: the rotation vector
    theta - a scalar
    """
    #YOUR CODE HERE

    theta =  2*np.arccos(rot[3])
    if theta==0:
        omega = np.array([0,0,0])
    else:
        omega = rot[0:3]/math.sin(theta/2)

    return (omega, theta)
    
def create_rbt(omega, theta, trans):
    """
    Creates a rigid body transform using omega, theta, and the translation component.
    g = [R,p; 0,1], where R = exp(omega * theta), p = trans
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray or 3x1 array: the translation component of the rigid body motion
    
    Returns:
    g - (4,4) ndarray : the rigid body transform
    """
    #YOUR CODE HERE
    R = robot206.rotation_3d(omega, theta)
    p = np.reshape(trans, (-1, 1))
    g_1 = np.hstack((R, p))
    g = np.vstack((g_1, np.array([0,0,0,1])))
    return g
    
def compute_gab(g0a,g0b):
    """
    Creates a rigid body transform g_{ab} the converts between frame A and B
    given the coordinate frame A,B in relation to the origin
    
    Args:
    g0a - (4,4) ndarray : the rigid body transform from the origin to frame A
    g0b - (4,4) ndarray : the rigid body transform from the origin to frame B
    
    Returns:
    gab - (4,4) ndarray : the rigid body transform
    """
    #YOUR CODE HERE
    gab = np.dot(np.linalg.inv(g0a), g0b)
    return gab
    
def find_omega_theta(R):
    """
    Given a rotation matrix R, finds the omega and theta such that R = exp(omega * theta)
    
    Args:
    R - (3,3) ndarray : the rotational component of the rigid body transform
    
    Returns:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    """
    #YOUR CODE HERE
    theta = np.arccos((np.trace(R)-1)/2)
    r_32 = R[2,1]
    r_23 = R[1,2]
    r_31 = R[2,0]
    r_13 = R[0,2]
    r_21 = R[1,0]
    r_12 = R[0,1]
    omega = 1/(2*math.sin(theta))*np.array([r_32-r_23, r_13-r_31, r_21-r_12])
    return (omega, theta)
    
def find_v(omega, theta, trans):
    """
    Finds the linear velocity term of the twist (v,omega) given omega, theta and translation
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray of 3x1 list : the translation component of the rigid body transform
    
    Returns:
    v - (3,1) ndarray : the linear velocity term of the twist (v,omega)
    """    
    #YOUR CODE HERE
    omega_hat = robot206.skew_3d(omega)
    A_1 = np.dot((np.eye(3)-robot206.rotation_3d(omega, theta)),omega_hat)
    A_2 = np.outer(omega, omega)*theta
    A = A_1 + A_2
    v = np.reshape(np.dot(np.linalg.inv(A),trans), (-1,1))
    return v
    
#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------

def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')
        
def array_func_test_two_outputs(func_name, args, ret_desireds):
    ret_values = func_name(*args)
    for i in range(2):
        ret_value = ret_values[i]
        ret_desired = ret_desireds[i]
        if i == 0 and not isinstance(ret_value, np.ndarray):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
        elif i == 1 and not isinstance(ret_value, float):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a float')
        elif i == 0 and ret_value.shape != ret_desired.shape:
            print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
        elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
            print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
        else:
            print('[PASS] ' + func_name.__name__ + '() returned the argument %d value!' % i)

if __name__ == "__main__":
    print('Testing...')
    
    #Test quaternion_to_exp()
    arg1 = np.array([1.0, 2, 3, 0.1])
    func_args = (arg1,)
    ret_desired = (np.array([1.005, 2.0101, 3.0151]), 2.94125)
    array_func_test_two_outputs(quaternion_to_exp, func_args, ret_desired)
    
    #Test create_rbt()
    arg1 = np.array([1.0, 2, 3])
    arg2 = 2
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(create_rbt, func_args, ret_desired)
    
    #Test compute_gab(g0a,g0b)
    g0a = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    g0b = np.array(
      [[-0.6949,  0.7135,  0.0893,  0.5   ],
       [-0.192 , -0.3038,  0.9332, -0.5   ],
       [ 0.693 ,  0.6313,  0.3481,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    func_args = (g0a, g0b)
    ret_desired = np.array([[-0.6949, -0.192 ,  0.693 ,  0.    ],
       [ 0.7135, -0.3038,  0.6313,  0.    ],
       [ 0.0893,  0.9332,  0.3481,  0.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(compute_gab, func_args, ret_desired)
    
    #Test find_omega_theta
    R = np.array(
      [[ 0.4078, -0.6562,  0.6349 ],
       [ 0.8384,  0.5445,  0.0242 ],
       [-0.3616,  0.5224,  0.7722 ]])
    func_args = (R,)
    ret_desired = (np.array([ 0.2673,  0.5346,  0.8018]), 1.2001156089449496)
    array_func_test_two_outputs(find_omega_theta, func_args, ret_desired)
    
    #Test find_v
    arg1 = np.array([1.0, 2, 3])
    arg2 = 1
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array([[-0.1255],
       [ 0.0431],
       [ 0.0726]])
    array_func_test(find_v, func_args, ret_desired)
