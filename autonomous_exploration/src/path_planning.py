#!/usr/bin/env python

import rospy
import math

# Class for reading the data from sensors
class PathPlanning:

    # Constructor
    def __init__(self):
        pass

    def createPath(self, ogm, robot_pose, target_pose):
        return self.aStar(ogm, robot_pose, target_pose)

    # The heuristic function for A*
    def aStarHeuristic(self, p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1])

    # Reconstruction of path for A*
    def aStarReconstructPath(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path

    def aStar(self, ogm, start, goal):

        # The set of evaluated nodes
        closed_set = []
        # The set of nodes to be evaluated
        open_set = []
        open_set.append(start)
        # The map of navigated nodes
        came_from = {}
        came_from[start] = None

        # Cost from path along best known path
        g_score = {}
        g_score[start] = 0
        # Estimated total cost from start to goal via y
        f_score = {}
        f_score[start] = g_score[start] + self.aStarHeuristic(start, goal) #??

        while len(open_set) != 0:
            # Find element in open set with minimum f_score
            min_key = -1
            min_val = math.inf
            for key in open_set:
                if f_score[key] < min_val:
                    min_key = key
                    min_val = f_score[key]

            current = min_key
            # Check if the goal was reached
            if current == goal:
                return self.aStarReconstructPath(came_from, goal)

            open_set.remove(current)
            closed_set.append(current)

            # Go through the current's neighbors
            for i in range(-1, 2):
                for j in range(-1, 2):
                    neighbor = [current[0] + i, current[1] + j]
                    # Check if the neighbor is unoccupied
                    if ogm[neighbor[0]][neighbor[1]] > 49:
                        continue
                    # The neighbor is already evaluated
                    if neighbor in closed_set:
                        continue
                    # Length of this path
                    tentative_g_score = g_score[current] + \
                            self.aStarHeuristic(current, neighbor)

                    # Check if we have a new node
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                    elif tentative_g_score >= g_score[neighbor]:
                        continue    # This is not a better path

                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + \
                            self.aStarHeuristic(neighbor, goal)
        
        # Path not found!
        return []
