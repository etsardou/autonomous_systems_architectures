#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from robot_perception import RobotPerception

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):

        self.print_velocities = False
        self.stop_robot = False

        self.sonar_aggregation = SonarDataAggregator()
        self.laser_aggregation = LaserDataAggregator()
        self.robot_perception  = RobotPerception()

        self.linear_velocity  = 0
        self.angular_velocity = 0

        # The timer produces events for sending the speeds every 110 ms
        rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
        self.velocity_publisher = rospy.Publisher("/robot0/cmd_vel", Twist,\
                queue_size = 10)

        self.velocity_arch = rospy.get_param("velocities_architecture")
        print "The selected velocities architecture is " + self.velocity_arch

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
        # Choose architecture
        if self.velocity_arch == "subsumption":
            self.produceSpeedsSubsumption()
        else:
            self.produceSpeedsMotorSchema()

        # Create the commands message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0 
        twist.angular.y = 0
        twist.angular.z = self.angular_velocity

        # Send the command
        self.velocity_publisher.publish(twist)

        if self.print_velocities == True:
          print "[L,R] = [" + str(twist.linear.x) + " , " + \
              str(twist.angular.z) + "]"

    # Produce speeds from sonars
    def produceSpeedsSonars(self):
        front   = self.sonar_aggregation.sonar_front_range
        left    = self.sonar_aggregation.sonar_left_range
        right   = self.sonar_aggregation.sonar_right_range
        r_left  = self.sonar_aggregation.sonar_rear_left_range
        r_right = self.sonar_aggregation.sonar_rear_right_range
        
        linear  = 0
        angular = 0

        # YOUR CODE HERE ------------------------------------------------------
        # Adjust the linear and angular velocities using the five sonars values

        # ---------------------------------------------------------------------
        return [linear, angular]

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
        scan   = self.laser_aggregation.laser_scan
        
        linear  = 0
        angular = 0

        # YOUR CODE HERE ------------------------------------------------------
        # Adjust the linear and angular velocities using the laser scan

        # ---------------------------------------------------------------------
        return [linear, angular]

    # Combines the speeds into one output using a subsumption approach
    def produceSpeedsSubsumption(self):
        [l_sonar, a_sonar] = self.produceSpeedsSonars()
        [l_laser, a_laser] = self.produceSpeedsLaser()

        self.linear_velocity  = 0
        self.angular_velocity = 0

        # YOUR CODE HERE ------------------------------------------------------
        
        # ---------------------------------------------------------------------

    # Combines the speeds into one output using a motor schema approach
    def produceSpeedsMotorSchema(self):
        [l_sonar, a_sonar] = self.produceSpeedsSonars()
        [l_laser, a_laser] = self.produceSpeedsLaser()

        self.linear_velocity  = 0
        self.angular_velocity = 0
        
        # YOUR CODE HERE ------------------------------------------------------
        
        # ---------------------------------------------------------------------

    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False;
