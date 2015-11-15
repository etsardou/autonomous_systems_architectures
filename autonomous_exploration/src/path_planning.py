#!/usr/bin/env python

import rospy
import math

# Class that implements path planning via A*
class PathPlanning:

    # Constructor
    def __init__(self):
        pass

    # Function to be called from navigation
    def createPath(self, ogm, robot_pose, target_pose):
        # Debugging purposes print
        print "PathPlanning: RobotPose=" + str(robot_pose) + " Target=" + \
                str(target_pose)
        # Call of the internal A* function
        return self.aStar(ogm, robot_pose, target_pose)

    # The heuristic function for A* - simply the Eucledian distance
    def aStarHeuristic(self, p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1])

    # Reconstruction of path for A*
    def aStarReconstructPath(self, came_from, current):
        total_path = [current]
        local_curr = list(current)
        # Following the path backwards
        while True:
            temp = came_from[tuple(local_curr)]
            if temp == None:
                break
            local_curr = list(temp)
            total_path.append(local_curr)
        return total_path

    # The A* implementation
    def aStar(self, ogm, start, goal):
        
        # The set of evaluated nodes
        closed_set = []
        # The set of nodes to be evaluated
        open_set = []
        open_set.append(tuple(start))
        # The map of navigated nodes
        came_from = {}
        came_from[tuple(start)] = None

        # Cost from path along best known path
        g_score = {}
        g_score[tuple(start)] = 0
        # Estimated total cost from start to goal via y
        f_score = {}
        f_score[tuple(start)] = g_score[tuple(start)] + \
                self.aStarHeuristic(tuple(start), tuple(goal))

        # Try while there are elements to check
        while len(open_set) != 0:
            # Find element in open set with minimum f_score
            min_key = -1
            min_val = float('inf')
            for key in open_set:
                if f_score[tuple(key)] < min_val:
                    min_key = key
                    min_val = f_score[tuple(key)]

            current = min_key
            # Check if the goal was reached
            if tuple(current) == tuple(goal):
                return self.aStarReconstructPath(came_from, goal)

            open_set.remove(tuple(current))
            closed_set.append(tuple(current))

            # Go through the current's neighbors
            for i in range(-1, 2):
                for j in range(-1, 2):
                    neighbor = [current[0] + i, current[1] + j]
                    # Check if the neighbor is unoccupied
                    if ogm[neighbor[0]][neighbor[1]] > 49:
                        continue
                    
                    # The neighbor is already evaluated
                    if tuple(neighbor) in closed_set:
                        continue
                    # Length of this path
                    tentative_g_score = g_score[tuple(current)] + \
                            self.aStarHeuristic(tuple(current), tuple(neighbor))
                    
                    # Check if we have a new node
                    if tuple(neighbor) not in open_set:
                        open_set.append(tuple(neighbor))
                    elif tentative_g_score >= g_score[tuple(neighbor)]:
                        continue    # This is not a better path

                    #print "Update values"
                    came_from[tuple(neighbor)] = current
                    g_score[tuple(neighbor)] = tentative_g_score
                    f_score[tuple(neighbor)] = g_score[tuple(neighbor)] + \
                            self.aStarHeuristic(tuple(neighbor), tuple(goal))
        
        # Path not found!
        return []
