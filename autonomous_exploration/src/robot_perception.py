#!/usr/bin/env python

import rospy
import tf
import numpy
from nav_msgs.msg import OccupancyGrid

# Class for reading the data from sensors
class RobotPerception:

    # Constructor
    def __init__(self):

        self.print_robot_pose = False

        self.ogm = 0
        self.resolution = 0.2
        
        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0

        # Initialization of robot pose
        # x,y are in meters
        # x_px, y_px are in pixels
        self.robot_pose = {}
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0
        self.robot_pose['x_px'] = 0
        self.robot_pose['y_px'] = 0

        # Use tf to read the robot pose
        self.listener = tf.TransformListener()

        # Read robot pose with a timer
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

        # ROS Subscriber to the occupancy grid map
        rospy.Subscriber("/slam/occupancyGridMap", OccupancyGrid, self.getMap) 

    # Getting data from the laser
    def readRobotPose(self, event):
        try:
            (translation, rotation) = self.listener.lookupTransform\
                    ("/map", "/base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Error in tf"
            return

        self.robot_pose['x'] = translation[0]
        self.robot_pose['y'] = translation[1]
        self.robot_pose['x_px'] = self.robot_pose['x'] / self.resolution
        self.robot_pose['y_px'] = self.robot_pose['y'] / self.resolution

        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robot_pose['th'] = angles[2]

        if self.print_robot_pose == True:
            print self.robot_pose

    # Getting the occupancy grid map
    def getMap(self, data):
        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 is the unknown
        self.ogm = numpy.array(data.data).reshape(\
                data.info.width, data.info.height)
        self.resolution = data.info.resolution
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y

