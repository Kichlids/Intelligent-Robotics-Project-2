#!/usr/bin/env python

import rospy

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
destinations = []


'''
[[Coord(2, 3), Coord(9, 8)],
 [Coord(12, 9), Coord(4, 14)]]
'''


class TaskPlanner():

    def plan_route(self, points):
        list_coords = points

        waypoints = []
        
        start = my_location        

        min_dist = float('inf')
        min_index = -1
        for i in range(len(list_coords)):

            dist = self.calculate_distance(start, list_coords[i][0])
            if dist < min_dist:
                min_dist = dist
                min_index = i

        waypoints.append(list_coords[min_index][0])
        waypoints.append(list_coords[min_index][1])

        start = list_coords[min_index][1]
        list_coords.pop(min_index)

        while len(list_coords) > 0:

            min_dist = float('inf')
            min_index = -1

            for i in range(len(list_coords)):
                dist = self.calculate_distance(start, list_coords[i][0])
                if dist < min_dist:
                    min_dist = dist
                    min_index = i
            
            waypoints.append(list_coords[min_index][0])
            waypoints.append(list_coords[min_index][1])
            start = list_coords[min_index][1]
            list_coords.pop(min_index)

        for i in range(len(waypoints)):
            waypoints[i].print_coord()

        return waypoints


    def calculate_distance(self, coord1, coord2):
        dist = ((coord1.x - coord2.x) ** 2 + (coord1.y - coord2.y) ** 2) ** (0.5)
        return dist








def init_control_node():
    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    planner = TaskPlanner()

    points = [[Coord(2, 3), Coord(9, 8)],
              [Coord(12, 9), Coord(4, 14)]]

    planner.plan_route(points)


if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass