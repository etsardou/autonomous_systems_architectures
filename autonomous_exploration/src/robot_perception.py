#!/usr/bin/env python

import rospy
import tf

# Class for reading the data from sensors
class RobotPerception:

    # Constructor
    def __init__(self):

        # Initialization of robot pose
        self.robot_pose = {}
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0

        # Use tf to read the robot pose
        self.listener = tf.TransformListener()

        # Read robot pose with a timer
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

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
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robot_pose['th'] = angles[2]
        print self.robot_pose

