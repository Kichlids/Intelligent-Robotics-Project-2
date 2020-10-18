#!/usr/bin/env python

import rospy
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# Speed ft/s
LINEAR_SPEED_DEFAULT = 0.5
# Rotation speed ft/s
ANGULAR_SPEED_DEFAULT = 3

class Coord():

    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def print_coord(self):
        print(str(self.x) + ', ' + str(self.y))


my_location = Coord(0, 0)
waypoints = []





class Plan():

    def plan_route(self, list_set):
        coord_pairs = list_set

        waypoints = []
        
        start = my_location        

        min_dist = float('inf')
        min_index = -1
        for i in range(len(coord_pairs)):

            dist = self.calculate_distance(start, coord_pairs[i][0])
            if dist < min_dist:
                min_dist = dist
                min_index = i

        waypoints.append(coord_pairs[min_index][0])
        waypoints.append(coord_pairs[min_index][1])

        start = coord_pairs[min_index][1]
        coord_pairs.pop(min_index)

        while len(coord_pairs) > 0:

            min_dist = float('inf')
            min_index = -1

            for i in range(len(coord_pairs)):
                dist = self.calculate_distance(start, coord_pairs[i][0])
                if dist < min_dist:
                    min_dist = dist
                    min_index = i
            
            waypoints.append(coord_pairs[min_index][0])
            waypoints.append(coord_pairs[min_index][1])
            
            start = coord_pairs[min_index][1]
            coord_pairs.pop(min_index)

        for i in range(len(waypoints)):
            waypoints[i].print_coord()

        return waypoints

    def calculate_distance(self, coord1, coord2):
        dist = ((coord1.x - coord2.x) ** 2 + (coord1.y - coord2.y) ** 2) ** (0.5)
        return dist


class Support():

    # Convert meters to feet
    def meters_to_feet(self, val):
        return val * 3.28

    # Convert feet to meters
    def feet_to_meters(self, val):
        return val / 3.28

    # Convert radians to degrees
    def rad_to_deg(self, rad):
        return rad * 180 / math.pi


class Navigation():

    def __init__(self):
        self.waypoint_index = 0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_rotation_callback)
        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.support = Support()
        self.yaw = 0



    def odom_rotation_callback(self, data):
        #print('odom callback')
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        #self.yaw = yaw
        #print(math.degrees(self.yaw))

        #print(math.degrees(self.yaw))


    def get_rotation_angle(self, destination):
        delta_x = destination.x - my_location.x
        delta_y = destination.y - my_location.y

        angle = 0

        if delta_x == 0:
            # Target ahead or behind us
            if delta_y > 0:
                angle = 0
            elif delta_y < 0:
                angle = 180
        elif delta_y == 0:
            # Target to the left or right of us
            if delta_x > 0:
                angle = -90
            elif delta_x < 0:
                angle = 90
        else:
            angle = math.degrees(math.atan2(delta_x, delta_y))
            if delta_y < 0:
                angle += 180
        
        return angle#math.radians(angle)


    def navigate(self, waypoints):
        # Assuming robot faces forward (+y direction) at 0,0 initially

        
        # Rotating
        t0 = 0
        t1 = 0
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = self.get_rotation_angle(waypoints[self.waypoint_index])
        
        print('Rotating ' + str(round(target_angle, 2)))

        # Determine the direction to turn
        turn_msg = Twist()
        if target_angle >= 0:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT

        print(current_angle)

        
        while (current_angle < abs(target_angle)):
            
            self.velocity_pub.publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = (ANGULAR_SPEED_DEFAULT) * (t1 - t0)        
            print(current_angle)
        





def init_control_node():
    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    planner = Plan()
    navigator = Navigation()

    points = [[Coord(2, 3), Coord(9, 8)],
              [Coord(12, 9), Coord(4, 14)]]

    waypoints = planner.plan_route(points)

    navigator.navigate(waypoints)


if __name__ == '__main__':
    init_control_node()
    '''
    try:
        
    except rospy.ROSInterruptException:
        pass
    '''