#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist


# Speed ft/s
LINEAR_SPEED_DEFAULT = 0.5
# Rotation speed ft/s
ANGULAR_SPEED_DEFAULT = 0.4

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


class Navigation():

    def __init__(self):
        waypoint_index = 0\

    def get_rotation_angle(self, destination):
        delta_x = destination.x - my_location.x
        delta_y = destination.y = my_location.y

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
                angle = 90
            elif delta_x < 0:
                angle = -90
        else:
            angle = math.atan2(delta_x, delta_

    def navigate(self, waypoints):
        # Assuming robot faces forward (+y direction) at 0,0 initially

        while waypoint_index < len(waypoints):

            # Turn
            delta_x = waypoints[waypoint_index].x - my_location.x
            delta_y = waypoints[waypoint_index].y - my_location.y
            

            if delta_x == 0:
                # Target ahead or behind us
                if delta_y > 0:
                    angle = 0
                elif delta_y < 0:
                    angle = 180
            
            elif delta_y == 0:
                # Target to the left or right of us
                if delta_x > 0:
                    angle = 90
                elif delta_x < 0:
                    angle = -90


            if delta_y < 0:







def init_control_node():
    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    planner = Plan()

    points = [[Coord(2, 3), Coord(9, 8)],
              [Coord(12, 9), Coord(4, 14)]]

    planner.plan_route(points)


if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass