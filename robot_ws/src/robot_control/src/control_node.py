#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Speed ft/s
LINEAR_SPEED_DEFAULT = 0.5
# Rotation speed rad ft/s 
ANGULAR_SPEED_DEFAULT = 0.1

class Coord():

    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def print_coord(self):
        return str(self.x) + ', ' + str(self.y)


my_location = Coord(0, 0)
waypoints = []


class Odom():
    def __init__(self):
        self.support = Support()
        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
    def odom_callback(self, data):
        global my_location

        x = self.support.meters_to_feet(round(data.pose.pose.position.x, 2))
        y = self.support.meters_to_feet(round(-data.pose.pose.position.y, 2))

        my_location = Coord(x, y)
        #print(my_location.print_coord())



class Plan():

    def __init__(self):
        self.support = Support()

    def plan_route(self, list_set):
        global my_location

        coord_pairs = list_set

        waypoints = []
        
        start = my_location        

        min_dist = float('inf')
        min_index = -1
        for i in range(len(coord_pairs)):

            dist = self.support.calculate_distance(start, coord_pairs[i][0])
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
                dist = self.support.calculate_distance(start, coord_pairs[i][0])
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



class Support():

    def calculate_distance(self, coord1, coord2):
        dist = ((coord1.x - coord2.x) ** 2 + (coord1.y - coord2.y) ** 2) ** (0.5)
        return dist

    # Convert meters to feet
    def meters_to_feet(self, val):
        return val * 3.28

    # Convert feet to meters
    def feet_to_meters(self, val):
        return val / 3.28


class Navigation():

    def __init__(self):
        self.waypoint_index = 0
        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.support = Support()


    def get_rotation_angle(self, destination):
        global my_location
        print('My Coordinate: ' + my_location.print_coord())

        delta_x = destination.x - my_location.x
        delta_y = destination.y - my_location.y

        #print('deltaX: ' + str(delta_x))

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
            if delta_x < 0:
                print('Target behind us')
                angle += 180
        
        return angle#math.radians(angle)


    def navigate(self, waypoints):
        global my_location

        # Assuming robot faces forward (+y direction) at 0,0 initially

        while self.waypoint_index < len(waypoints):

            rospy.sleep(1)

            print('Rotating to ' + waypoints[self.waypoint_index].print_coord())

            # Rotating
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            target_angle = -self.get_rotation_angle(waypoints[self.waypoint_index])
            
            print('Rotating ' + str(round(target_angle, 2)))

            # Determine the direction to turn
            turn_msg = Twist()
            if target_angle >= 0:
                turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
            else:
                turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT

            while current_angle < abs(target_angle):
                
                self.velocity_pub.publish(turn_msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = math.degrees(ANGULAR_SPEED_DEFAULT) * (t1 - t0)  
                #print(current_angle)      

            #my_location = waypoints[self.waypoint_index]
            rospy.sleep(1)
            
            # Move

            dist_diff = self.support.calculate_distance(my_location, waypoints[self.waypoint_index])
            while dist_diff > 1:
                vel_msg = Twist()
                vel_msg.linear.x = self.support.feet_to_meters(LINEAR_SPEED_DEFAULT)
                self.velocity_pub.publish(vel_msg)
                dist_diff = self.support.calculate_distance(my_location, waypoints[self.waypoint_index])
                print('Dist diff: ' + str(dist_diff))

            self.waypoint_index += 1
            rospy.sleep(1)
            
        
        





def init_control_node():
    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    planner = Plan()
    navigator = Navigation()
    odom = Odom()

    points = [[Coord(2, 3), Coord(9, 8)],
              [Coord(12, 9), Coord(4, 14)]]

    waypoints = planner.plan_route(points)

    navigator.navigate(waypoints)


if __name__ == '__main__':
    
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass