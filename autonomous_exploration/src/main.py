#!/usr/bin/env python

import rospy

from speeds_assignment import RobotController

# The main function of the program
if __name__ == '__main__':

    # Initializes the ROS node
    rospy.init_node('robot_controller')
    # Creates a RobotController object
    rc = RobotController()
    # ROS waits for events
    rospy.spin()
