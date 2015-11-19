#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
from ar_coord.msg import ZumyCoord
import robot206
#import exp_quat_func as eqf

listener = None

def get_x_angle(trans,rot):
    g=return_rbt(trans,rot)
    x_direction = g[0:3,0]
    #print x_direction
    x_angle = math.atan2(x_direction[1],x_direction[0])
    return x_angle


def return_rbt(trans, rot):
    """
    Prints out the 4x4 rigid body transformation matrix from quaternions

    Input:
        (3,) array - translation ector
        (4,) array - rotation vector in quaternions
    """
    (omega,theta) = robot206.quaternion_to_exp(rot)
    g = robot206.create_rbt(omega,theta,trans)

    return g

# def compute_twist(rbt):
#     """
#     Computes the corresponding twist for the rigid body transform

#     Input:
#         rbt - (4,4) ndarray 

#     Output:
#         v - (3,) array
#         w - (3,) array
#     """
#     #YOUR CODE HERE

#     (omega,theta) = eqf.find_omega_theta(rbt[0:3,0:3])
#     v = eqf.find_v(omega, theta, rbt[0:3,3])

#     return (v,omega)

if __name__=='__main__':
    #Set up new node called 'publish_zumy_coord':
    rospy.init_node('publish_AR_coord')
    #Set up a publisher called 'zumy_position'

    
    #The input should have at least 4 additional tag number. First three is coord base
    #Starting from the 4-th one, is the tag's coord.
    if len(sys.argv) < 4:
        print('Use: publish_zumy_coord.py [ AR tag number ] [ AR tag number ] [ AR tag number ] [ AR tag number ]...')
        sys.exit()
    ar_suffix = 'ar_marker_'
    ar_tag_names = [ar_suffix+sys.argv for sys.argv in sys.argv[1:]]
    ar_origin = ar_tag_names[0]
    ar_x = ar_tag_names[1]
    ar_y = ar_tag_names[2]
    trans = {}
    rot = {}
    latest_t = {}

    listener = tf.TransformListener()
    #Set the publish rate to be 10Hz
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for curr_ar_tag in ar_tag_names[1:]:
            try:
                latest_t[curr_ar_tag] = listener.getLatestCommonTime(curr_ar_tag, ar_origin)
                #print curr_ar_tag
                (trans[curr_ar_tag], rot[curr_ar_tag]) = listener.lookupTransform(
                    ar_origin, curr_ar_tag, latest_t[curr_ar_tag])

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

                    numpy_trans = np.asarray(trans[curr_ar_tag])
                    numpy_rot = np.asarray(rot[curr_ar_tag])
                    curr_theta = get_x_angle(numpy_trans,numpy_rot)
                    curr_theta = curr_theta*180/np.pi;
                    # curr_theta = 2.0 * np.arccos(rot[curr_ar_tag][3])\
                    #     * np.sign(rot[curr_ar_tag][2])

                    curr_msg = ZumyCoord()
                    curr_msg.zumyID = curr_ar_tag
                    curr_msg.time.data = latest_t[curr_ar_tag]
                    curr_msg.position.x = curr_coord[0]
                    curr_msg.position.y = curr_coord[1]
                    curr_msg.position.theta = curr_theta
                    zumy_coord_pub = rospy.Publisher("/"+ curr_ar_tag +"/AR_position", ZumyCoord, queue_size=10)
                    zumy_coord_pub.publish(curr_msg)
                    print curr_ar_tag
                    print curr_coord
                    print curr_theta
                    print latest_t[curr_ar_tag]
        rate.sleep()



        # if trans_x_flag and trans_y_flag:
        #     base_coord_pub = BaseCoord()
        #     base_coord_pub.x_axis.x = trans_x[0]
        #     base_coord_pub.x_axis.y = trans_x[1]
        #     base_coord_pub.x_axis.z = trans_x[2]
        #     base_coord_pub.y_axis.x = trans_y[0]
        #     base_coord_pub.y_axis.y = trans_y[1]
        #     base_coord_pub.y_axis.z = trans_y[2]
        #     coord_pub.publish(base_coord_pub)
        
        


