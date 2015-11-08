#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        self.sonar_aggregation = SonarDataAggregator()
        self.laser_aggregation = LaserDataAggregator()

