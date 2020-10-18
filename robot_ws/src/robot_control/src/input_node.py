#! /usr/bin/env python

import rospy






def init_input_node():
    rospy.init_node('input_node', anonymous = False)
    rate = rospy.Rate(10)

if __name__ == '__main__':
    try:
        init_input_node()
    except rospy.ROSInterruptException:
        pass