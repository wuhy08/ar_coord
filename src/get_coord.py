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


if __name__=='__main__':
    if len(sys.argv) < 2:
        print('Use: get_coord.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
        sys.exit()
    ar_suffix = 'ar_marker_'
    ar_tag_names = [ar_suffix+sys.argv for sys.argv in sys.argv[1:]]
	rospy.init_node('get_coord')
	base_coord_sub = rospy.Subscriber('base_coord', BaseCoord)