#!/usr/bin/env python

import rospy

# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self):
        pass

    def selectTarget(self, ogm, coverage, robot_pose):
        
        # The next target in pixels
        next_target = [0, 0] 
        
        # YOUR CODE HERE ------------------------------------------------------
        # Here you must select the next target of the robot. The next target
        # should exist in unoccupied and uncovered space. Thus, you must use the
        # ogm and coverage variables or / and the robot pose. The easier way is to
        # randomly select points of the map until one such point can be a target
        # Of course you should try something smarter...!

        # ---------------------------------------------------------------------

        return next_target

