#!/usr/bin/env python
"""ROS206 is a self made module to support the calculation in EE206A course.
This module includes following functions:
    skew_3d
    rotation_2d
    rotation_3d
    hat_2d
    hat_3d
    homog_2d
    homog_3d
    prod_exp
"""

import numpy as np
#import scipy as sp
import math

from numpy import linalg

np.set_printoptions(precision=4,suppress=True)

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    #YOUR CODE HERE
    
    omega_hat = np.array([[0,-omega[2],omega[1]],
                         [omega[2],0,-omega[0]],
                         [-omega[1],omega[0],0]])

    return omega_hat

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """
    
    #YOUR CODE HERE
    
    rot = np.array([[np.cos(theta),-np.sin(theta)],
                    [np.sin(theta),np.cos(theta)]])
    
    return rot

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    #YOUR CODE HERE
    
    omega_hat = skew_3d(omega)
    omega_norm = linalg.norm(omega)
    R_1 = np.eye(3)
    R_2 = np.sin(omega_norm*theta)/omega_norm*omega_hat
    R_3 = np.dot(omega_hat, omega_hat)*(1-np.cos(omega_norm*theta))/omega_norm**2
    rot = R_1+R_2+R_3  

    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    #YOUR CODE HERE
    
    xi_hat = np.array([[0, -xi[2], xi[0]],
                      [xi[2], 0, xi[1]],
                      [0, 0, 0]])

    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    
    xi_hat = np.array([[0, -xi[5], xi[4], xi[0]],
              [xi[5], 0, -xi[3], xi[1]],
              [-xi[4], xi[3], 0, xi[2]],
              [0, 0, 0, 0]])

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    #YOUR CODE HERE
    
    R = rotation_2d(xi[2]*theta)
    p_a = np.eye(2)-R
    p_b = np.array([[0,-1],[1,0]])
    p_c = np.array([[xi[0]/xi[2]],[xi[1]/xi[2]]])
    p = np.dot(np.dot(p_a, p_b), p_c)
    g = np.eye(3)
    g[0:2, 0:2] = R
    g[0:2,2] = np.transpose(p)

    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    omega = xi[3:6]
    v = xi[0:3]

    omega_norm = linalg.norm(omega)
    if omega_norm != 0:
        R = rotation_3d(omega,theta)
        omega_hat = skew_3d(omega)
        p_a = np.dot(np.eye(3)-R, np.dot(omega_hat, v))/omega_norm**2
        p_b = np.dot(np.outer(omega, omega), v)*theta/omega_norm**2
        p = p_a+p_b
        g = np.eye(4)
        g[0:3,0:3] = R
        g[0:3,3] = np.transpose(p)
    else:
        g = np.eye(4)
        g[0:3,3] = np.transpose(v*theta)  

    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    #YOUR CODE HERE
    dim = theta.shape[0]
    if not dim == 1:
        g = np.dot(prod_exp(xi[:, 0:-1], theta[0:-1]), homog_3d(xi[:,-1], theta[-1]))
    else:
        g = homog_3d(xi.reshape(6), theta)

    return g
    
def twist_rot(omega, q):
    """
    Computes the twist, given the omega and q, in special case: rotation.
    
    Args:
    omega - (3,) ndarray: the direction of the rotation axis. Must be non-zero
    q     - (3,) ndarray: the displacement of each joint
    
    Returns:
    xi    - (6,) ndarray: the resulting homogeneous transformation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3x1')
    if not q.shape == (3,):
        raise TypeError('q must be a 3x1')
    omega_norm = linalg.norm(omega)
    if omega_norm == 0:
        raise TypeError('omega norm must be larger than 0')
        
    v = -np.dot(skew_3d(omega), q)
    xi = np.hstack((v,omega))
    
    return xi
    
def twist_trans(v):
    """
    Computes the twist, given the v, in special case: translational.
    
    Args:
    v - (3,) ndarray: the translational vector
    
    Returns:
    xi    - (6,) ndarray: the resulting homogeneous transformation matrix
    """
    if not v.shape == (3,):
        raise TypeError('v must be a 3x1')
    xi = np.hstack((v,np.array([0, 0, 0])))
    
    return xi
    
def sol_PK1(u, v, omega, tol):
    """
    Computes the first Paden-Kahan problem, given the u, v, omega and tol.
    The first P-K problem is described as:
    given vectors u and v of same length and same angle with omega,
    and omega with unit magnitude as axis,
    find theta such that: exp(hat(omega)*theta)*u=v
    
    Args:
    u - (3,) ndarray: vector before rotation
    v - (3,) ndarray: vector after rotation
    omega - (3,) ndarray: the rotation axis
    tol - float: the tolerance of the criteria
    
    Returns:
    theta - float: the solution of the P-K Prob 1
    """
    if not u.shape == (3,):
        raise TypeError('u must be a 3x1')
    if not v.shape == (3,):
        raise TypeError('v must be a 3x1')    
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3x1')
            
    norm_u = linalg.norm(u)
    norm_v = linalg.norm(v)
    proj_u = np.dot(omega, u)
    proj_v = np.dot(omega, v)
    if abs(norm_u-norm_v)>tol:
        print abs(norm_u-norm_v), '>',tol
        raise ValueError('difference of magnitude between u and v larger than tolerance')
    elif abs(proj_u-proj_v)>tol:
        print abs(proj_u-proj_v), '>',tol
        raise ValueError('difference of projection betweem u and v larger than tolerance')
    else:
        norm_omega = linalg.norm(omega)
        omega_p = omega/norm_omega
        u_p = u - np.dot(omega_p, u)*omega_p
        v_p = v - np.dot(omega_p, v)*omega_p
        theta = math.atan2(np.dot(omega_p, np.cross(u_p, v_p)), np.dot(u_p, v_p))
        
    return theta
    
def sol_PK2(u, v, omega1, omega2, tol):
    """
    Computes the second Paden-Kahan problem, given the u, v, omega1, omega2 and tol.
    The second P-K problem is described as:
    given vectors u and v of same length,
    and omega1 and omega2 with unit magnitude as axis,
    find theta such that: exp(hat(omega1)*theta1)*exp(hat(omega2)*theta2)*u=v
    
    Args:
    u - (3,) ndarray: vector before rotation
    v - (3,) ndarray: vector after rotation
    omega1 - (3,) ndarray: the rotation axis 1
    omega2 - (3,) ndarray: the rotation axis 2
    tol - float: the tolerance of the criteria
    
    Returns:
    theta - nx2 float: the solution of the P-K Prob 2, n=0, 1, or 2
    z
    """    
    if not u.shape == (3,):
        raise TypeError('u must be a 3x1')
    if not v.shape == (3,):
        raise TypeError('v must be a 3x1')    
    if not omega1.shape == (3,):
        raise TypeError('omega1 must be a 3x1')    
    if not omega2.shape == (3,):
        raise TypeError('omega2 must be a 3x1')    
    
    norm_u = linalg.norm(u)
    norm_v = linalg.norm(v)
    if abs(norm_u-norm_v)>tol:
        print abs(norm_u-norm_v), '>',tol
        raise ValueError('difference of magnitude between u and v larger than tolerance')
    else:
        norm_omega1 = linalg.norm(omega1)
        w1 = omega1/norm_omega1
        norm_omega2 = linalg.norm(omega2)
        w2 = omega2/norm_omega2
        alpha = (np.dot(w1, w2)*np.dot(w2, u)-np.dot(w1, v))/(np.dot(w1, w2)**2-1)
        beta = (np.dot(w1, w2)*np.dot(w1, v)-np.dot(w2, u))/(np.dot(w1, w2)**2-1)
        gamma2num = linalg.norm(u)**2-alpha**2-beta**2-2*alpha*beta*np.dot(w1,w2)
        gamma2den = linalg.norm(np.cross(w1,w2))**2
        gamma2 = gamma2num/gamma2den
        if gamma2 < 0:
            theta = np.array([[]])
        elif gamma2 == 0:
            z = alpha*w1+beta*w2
            theta2 = sol_PK1(u, z, w2, tol)
            theta1 = sol_PK1(z, v, w1, tol)
            theta = np.array([theta1, theta2])
        else:
            gamma = np.array([1, -1])*math.sqrt(gamma2)
            z = alpha*w1+beta*w2+np.outer(gamma, np.cross(w1, w2))
            theta = np.zeros([2,2])
            for i in range(2):
                theta2 = sol_PK1(u, z[i], w2, tol)
                theta1 = sol_PK1(z[i], v, w1, tol)
                theta[i, :] = np.array([theta1, theta2])
        
    return theta, z
    
def sol_PK3(u, v, omega, delta, tol):
    """
    Computes the third Paden-Kahan problem, given the u, v, omega, delta and tol.
    The second P-K problem is described as:
    given vectors u and v,
    omega with unit magnitude as axis,
    and delta a real number>0
    find theta such that: 
    norm( v - exp( hat( omega ) * theta ) * u ) = delta 
    
    Args:
    u - (3,) ndarray: vector before rotation
    v - (3,) ndarray: vector after rotation
    omega - (3,) ndarray: the rotation axis
    delta - float: the radius
    tol - float: the tolerance of the criteria
    
    Returns:
    theta - (n,) array: the solution of the P-K Prob 3, n =0,1 or 2
    """    
    if not u.shape == (3,):
        raise TypeError('u must be a 3x1')
    if not v.shape == (3,):
        raise TypeError('v must be a 3x1')    
    if not omega.shape == (3,):
        raise TypeError('omega1 must be a 3x1')    
    if not delta > 0:
        raise ValueError('delta must be greater than 0')    
    norm_omega = linalg.norm(omega)
    omega = omega/norm_omega
    u_p = u - np.dot(omega, u)*omega
    v_p = v - np.dot(omega, v)*omega
    delta_p_sq = delta**2 - np.dot(omega, u-v)**2
    theta_0 = math.atan2(np.dot(omega, np.cross(u_p, v_p)), np.dot(u_p, v_p))
    u_p_norm = linalg.norm(u_p)
    v_p_norm = linalg.norm(v_p)
    cos_theta_1 = (u_p_norm**2 + v_p_norm**2 - delta_p_sq)/(2*u_p_norm*v_p_norm)
    if cos_theta_1 > 1 or cos_theta_1<-1:
        theta = np.array([])
    elif cos_theta_1 == 1 or cos_theta_1 == -1:
        theta = np.array([theta_0 + math.acos(cos_theta_1)])
        while theta>math.pi:
            theta = theta-2*math.pi
        while theta<-math.pi:
            theta = theta+2*math.pi
    else:
        theta = theta_0 + np.array([-1,1])*math.acos(cos_theta_1)
        for i in range(2):
            while theta[i]>math.pi:
                theta[i] = theta[i]-2*math.pi
            while theta[i]<-math.pi:
                theta[i] = theta+2*math.pi           
        
    return theta


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
    R = rotation_3d(omega, theta)
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
    omega_hat = skew_3d(omega)
    A_1 = np.dot((np.eye(3)-rotation_3d(omega, theta)),omega_hat)
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

    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_2d()
    arg1 = 2.658
    func_args = (arg1,)
    ret_desired = np.array([[-0.8853, -0.465 ],
                            [ 0.465 , -0.8853]])
    array_func_test(rotation_2d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_2d()
    arg1 = np.array([2.0, 1, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                         [ 3.,  0.,  1.],
                         [ 0.,  0.,  0.]])
    array_func_test(hat_2d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_2d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.3924, -0.9198,  0.1491],
                         [ 0.9198, -0.3924,  1.2348],
                         [ 0.    ,  0.    ,  1.    ]])
    array_func_test(homog_2d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)
    
    #Test homog_3d() translation
    arg1 = np.array([2.0, 1, 3, 0, 0, 0])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 1.    ,  0.    ,  0.    ,  1.3160],
                            [ 0.    ,  1.    ,  0.    ,  0.6580],
                            [ 0.    ,  0.    ,  1.    ,  1.9740],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)
    
    #Test twist_rot()
    arg1 = np.array([0, 0, 1])
    arg2 = np.array([0, 7, 5])
    func_args = (arg1,arg2)
    ret_desired = np.array([ 7,  0,  0,  0, 0, 1])
    array_func_test(twist_rot, func_args, ret_desired)
    
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
    
    print('Done!')
