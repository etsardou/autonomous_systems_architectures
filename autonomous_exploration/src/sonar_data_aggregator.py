#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range

# Class for reading the data from sensors
class SonarDataAggregator:

    # Constructor
    def __init__(self):

        # Initialization of 
        self.sonar_front_range = 0
        self.sonar_left_range = 0
        self.sonar_right_range = 0
        self.sonar_rear_left_range = 0
        self.sonar_rear_right_range = 0

        # ROS Subscribers to the robot's sonars
        rospy.Subscriber("/robot0/sonar_0", Range, self.getDataSonarFront) 
        rospy.Subscriber("/robot0/sonar_1", Range, self.getDataSonarLeft) 
        rospy.Subscriber("/robot0/sonar_2", Range, self.getDataSonarRight) 
        rospy.Subscriber("/robot0/sonar_3", Range, self.getDataSonarRearLeft) 
        rospy.Subscriber("/robot0/sonar_4", Range, self.getDataSonarRearRight) 

    # Getting data from the front sonar
    def getDataSonarFront(self, data):
        if data.range == float('Inf'):
            self.sonar_front_range = data.max_range
        elif data.range == -float('Inf'):
            self.sonar_front_range = data.min_range
        else:
            self.sonar_front_range = data.range

    # Getting data from the left sonar
    def getDataSonarLeft(self, data):
        if data.range == float('Inf'):
            self.sonar_left_range = data.max_range
        elif data.range == -float('Inf'):
            self.sonar_left_range = data.min_range
        else:
            self.sonar_left_range = data.range

    # Getting data from the right sonar
    def getDataSonarRight(self, data):
        if data.range == float('Inf'):
            self.sonar_right_range = data.max_range
        elif data.range == -float('Inf'):
            self.sonar_right_range = data.min_range
        else:
            self.sonar_right_range = data.range

    # Getting data from the rear left sonar
    def getDataSonarRearLeft(self, data):
        if data.range == float('Inf'):
            self.sonar_rear_left_range = data.max_range
        elif data.range == -float('Inf'):
            self.sonar_rear_left_range = data.min_range
        else:
            self.sonar_rear_left_range = data.range

    # Getting data from the rear right sonar
    def getDataSonarRearRight(self, data):
        if data.range == float('Inf'):
            self.sonar_rear_right_range = data.max_range
        elif data.range == -float('Inf'):
            self.sonar_rear_right_range = data.min_range
        else:
            self.sonar_rear_right_range = data.range
