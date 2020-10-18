#! /usr/bin/env python
import rospy
import re

from robot_msgs.msg import coordinate
from robot_msgs.msg import tasks    

def init_input_node():
    rospy.init_node('input_node', anonymous = False)
    rate = rospy.Rate(10)

    global tasks_pub

    tasks_pub = rospy.Publisher('/robot/tasks', tasks, queue_size = 10)
    #keyboard_sub = rospy.Subscriber('/robot/some_bool_to_tell_us_to_get_input', bool, keyboard_callback)

    myTasks = tasks()

    print "Input coordinates:"
    string_input = raw_input()
    while string_input != "":
        
        temp = re.findall(r'\d+', string_input)
        nums = list(map(float, temp)) 

        if len(nums) == 4:
            start_coord = coordinate()
            end_coord = coordinate()
            start_coord.x_coord = nums[0]
            start_coord.y_coord = nums[1]
            end_coord.x_coord = nums[2]
            end_coord.y_coord = nums[3]

            myTasks.coord_list.append(start_coord)
            myTasks.coord_list.append(end_coord)
        else:
            print "Input not valid as (<start_coordinates>, <end_coordinates>)"
        string_input = raw_input()

    print myTasks
    tasks_pub.publish(myTasks)

if __name__ == '__main__':
    try:
        init_input_node()
    except rospy.ROSInterruptException:
        pass