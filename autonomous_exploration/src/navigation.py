#!/usr/bin/env python

import rospy
import math
from robot_perception import RobotPerception
from target_selection import TargetSelection
from path_planning import PathPlanning

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Class for reading the data from sensors
class Navigation:

    # Constructor
    def __init__(self):

        # Initializations
        self.robot_perception = RobotPerception()
        self.target_selection = TargetSelection()
        self.path_planning = PathPlanning()

        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # Flag to check if the vehicle has a target or not
        self.target_exists = False
        self.inner_target_exists = False

        # Container for the current path
        self.path = []
        # Container for the subgoals in the path
        self.subtargets = []

        # Container for the next subtarget. Holds the index of the next subtarget
        self.next_subtarget = 0

        # Check if subgoal is reached 
        rospy.Timer(rospy.Duration(0.10), self.checkTarget)

        # ROS Publisher for the path
        self.path_publisher = rospy.Publisher("/autonomous_systems/path", \
            Path, queue_size = 10)

    def checkTarget(self, event):
        # Check if we have a target or if the robot just wanders
        if self.inner_target_exists == False or self.move_with_target == False:
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
        # IMPORTANT: The robot must be stopped if you call this function until
        # it is over
        # Check if we have a map
        while self.robot_perception.have_map == False:
          return

        print "Navigation: Producing new target"
        # We are good to continue the exploration
        # Make this true in order not to call it again from the speeds assignment
        self.target_exists = True

        # Manually update the coverage field
        self.robot_perception.updateCoverage()
        print "Navigation: Coverage updated"
      
        local_ogm = self.robot_perception.getMap()
        local_coverage = self.robot_perception.getCoverage()
        print "Got the map and Coverage"
  
        # Call the target selection function to do this
        target = self.target_selection.selectTarget(\
            local_ogm,\
            local_coverage,\
            self.robot_perception.robot_pose)
        print "Navigation: New target: " + str(target)

        # Once the target has been found, find the path to it
        # Get the global robot pose
        g_robot_pose = self.robot_perception.getGlobalCoordinates(\
            [self.robot_perception.robot_pose['x_px'],\
            self.robot_perception.robot_pose['y_px']])

        #target = [242,250]
        self.path = self.path_planning.createPath(\
            local_ogm,\
            g_robot_pose,\
            target)
        print "Navigation: Path for target found with " + str(len(self.path)) +\
            " points"

        # Break the path to subgoals every 25 pixels (0.5m)
        n_subgoals = (int) (len(self.path)/25)
        self.subtargets = []
        for i in range(0, n_subgoals):
          self.subtargets.append(self.path[i * 25])
        self.subtargets.append(self.path[-1])
        self.next_subtarget = 0

        # Publish the target and the path for visualization purposes
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in self.path:
          ps = PoseStamped()
          ps.header.frame_id = "map"
          ps.pose.position.x = p[0] * self.robot_perception.resolution + self.robot_perception.origin['x']
          ps.pose.position.y = p[1] * self.robot_perception.resolution + self.robot_perception.origin['y']
          ros_path.poses.append(ps)

        self.path_publisher.publish(ros_path)

        self.inner_target_exists = True

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

