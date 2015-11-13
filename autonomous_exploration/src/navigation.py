#!/usr/bin/env python

import rospy
import math
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

        # Container for the current path
        self.path = []
        # Container for the subgoals in the path
        self.subtargets = []

        # Container for the next subtarget. Holds the index of the next subtarget
        self.next_subtarget = 0

        # Check if subgoal is reached 
        rospy.Timer(rospy.Duration(0.10), self.checkTarget)

    def checkTarget(self, event):
        # Check if we have a target
        if self.target_exists == False:
          return

        # Get the robot pose in pixels
        [rx, ry] = [\
            self.robot_perception.robot_pose['x_px'],
            self.robot_perception.robot_pose['y_px']]
        # Find the distance between the robot pose and the next subtarget
        dist = math.hypot(\
            rx - self.subtargets[self.next_subtarget][0], \
            ry - self.subtargets[self.next_subtarget][1])

        # Check if distance is less than 10 px (20 cm)
        if dist < 10:
          self.next_subtarget += 1

          # Check if the final subtarget has been approached
          if self.next_subtarget == len(self.subtargets):
            self.target_exists = False

    def selectTarget(self):

        # Manually update the coverage field
        self.robot_perception.updateCoverage()
      
        # Call the target selection function to do this
        self.target_selection.selectTarget(\
            self.robot_perception.ogm,\
            self.robot_perception.coverage,\
            self.robot_perception.robot_pose)

        # Once the target has been found, find the path to it
        self.path = self.createPath()
        # Break the path to subgoals every 25 pixels (0.5m)
        n_subgoals = (int) (len(self.path)/25)
        self.subtargets = []
        for i in range(0, n_subgoals):
          self.subtargets.append(self.path[i * 25])
        self.subtargets.append(self.path[-1])
        self.next_subtarget = 0

        # We are good to continue the exploration
        self.target_exists = True

    def velocitiesToNextSubtarget(self):
        
        [linear, angular] = [0, 0]

        # YOUR CODE HERE ------------------------------------------------------
        # The velocities of the robot regarding the next subtarget should be 
        # computed. The known parameters are the robot pose [x,y,th] from 
        # robot_perception and the next_subtarget [x,y]. From these, you can 
        # compute the robot velocities for the vehicle to approach the target.
        # Hint: Trigonometry is required

        # ---------------------------------------------------------------------

        return [linear, angular]

