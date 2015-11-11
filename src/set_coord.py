#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
from ar_coord.msg import BaseCoord
import exp_quat_func as eqf

listener = None

def return_rbt(trans, rot):
    """
    Prints out the 4x4 rigid body transformation matrix from quaternions

    Input:
        (3,) array - translation ector
        (4,) array - rotation vector in quaternions
    """

    #YOUR CODE HERE
    (omega,theta) = eqf.quaternion_to_exp(rot)
    g = eqf.create_rbt(omega,theta,trans)

    return g

def compute_twist(rbt):
    """
    Computes the corresponding twist for the rigid body transform

    Input:
        rbt - (4,4) ndarray 

    Output:
        v - (3,) array
        w - (3,) array
    """
    #YOUR CODE HERE

    (omega,theta) = eqf.find_omega_theta(rbt[0:3,0:3])
    v = eqf.find_v(omega, theta, rbt[0:3,3])

    return (v,omega)

if __name__=='__main__':
    rospy.init_node('set_coord')
    if len(sys.argv) < 4:
        print('Use: ser_coord.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_suffix = 'ar_marker_'
    ar_tag_names = [ar_suffix+sys.argv for sys.argv in sys.argv[1:]]
    ar_origin = ar_tag_names[0]
    ar_x = ar_tag_names[1]
    ar_y = ar_tag_names[2]
    trans = {}
    rot = {}

    #     ar_tags = {}
    # ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
    # ar_tags['arx'] = 'ar_marker_' + sys.argv[2]
    # ar_tags['ary'] = 'ar_marker_' + sys.argv[3]

    listener = tf.TransformListener()
    #coord_pub = rospy.Publisher('base_coord', BaseCoord, queue_size=10)


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for curr_ar_tag in ar_tag_names[1:]:
            try:
                latest_t = listener.getLatestCommonTime(curr_ar_tag, ar_origin)
                #print curr_ar_tag
                (trans[curr_ar_tag], rot[curr_ar_tag]) = listener.lookupTransform(
                    ar_origin, curr_ar_tag, latest_t)

                #print trans[curr_ar_tag]
            except (tf.LookupException, tf.Exception):
                print 'Cannot get TF in '+curr_ar_tag
        if ar_x in trans and ar_y in trans:
            base_coord_x = np.array(trans[ar_x])
            #print "x"
            #print base_coord_x
            base_coord_y = np.array(trans[ar_y])
            #print "y"
            #print base_coord_y
            base_coord_z = np.cross(base_coord_x, base_coord_y)
            base_coord_matrix = np.vstack((base_coord_x, base_coord_y, base_coord_z))
            #print base_coord_matrix
            base_coord_matrix_inv = np.linalg.inv(base_coord_matrix)
            for curr_ar_tag in ar_tag_names[3:]:
                if curr_ar_tag in trans:
                    curr_trans = np.array(trans[curr_ar_tag])
                    curr_coord = np.dot(curr_trans, base_coord_matrix_inv)
                    curr_theta = 2.0*np.arccos(rot[curr_ar_tag][3])*np.sign(rot[curr_ar_tag][2])
                    print curr_ar_tag
                    print curr_coord
                    print curr_theta*180/np.pi




        # if trans_x_flag and trans_y_flag:
        #     base_coord_pub = BaseCoord()
        #     base_coord_pub.x_axis.x = trans_x[0]
        #     base_coord_pub.x_axis.y = trans_x[1]
        #     base_coord_pub.x_axis.z = trans_x[2]
        #     base_coord_pub.y_axis.x = trans_y[0]
        #     base_coord_pub.y_axis.y = trans_y[1]
        #     base_coord_pub.y_axis.z = trans_y[2]
        #     coord_pub.publish(base_coord_pub)
        
        

        rate.sleep()
