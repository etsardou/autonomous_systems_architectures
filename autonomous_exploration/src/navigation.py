#!/usr/bin/env python

import rospy
from robot_perception import RobotPerception
from target_selection import TargetSelection
from path_planning import PathPlanning

# Class for reading the data from sensors
class Navigation:

    # Constructor
    def __init__(self):

        # Initializations
        self.robot_perception = RobotPerception()
        self.target_selection = TargetSelection()
        self.path_planning = PathPlanning()

        # Flag to check if the vehicle has a target or not
        self.target_exists = False

        # Check if subgoal is reached 
        rospy.Timer(rospy.Duration(0.11), self.checkTarget)

    def checkTarget(self, event):
      pass  

    def selectTarget(self):
      self.target_exists = True
      pass

    def velocitiesToTarget(self):
      pass

    def createPath(self):
      pass
