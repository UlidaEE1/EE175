#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
#from motion_planning1 import get_waypoint_path



def heuristic_cost_to_go(neighbor, goal):
    #implement taxi cab distance to obtain heuristic
    hueristic_cost = abs(goal[0]-neighbor[0]) + abs(goal[1]-neighbor[1])
    return hueristic_cost

def get_neighbors(current):
    #find the four neighbors
    neighbors = [(current[0]+1,current[1]),(current[0]-1,current[1]),(current[0],current[1]+1),(current[0],current[1]-1)]
    return neighbors

def assemble_path(parents,current,start,goal):
    path = [goal]
    path.insert(0,current)
    temp_current = current
    while parents[temp_current] != start:
        parent_curr = parents[temp_current]
        path.insert(0,parent_curr)
        temp_current = parents[temp_current]
    return path

def get_waypoint_path(start, goal):
	obstacles = [(0,0),(1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0),(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7),(1,7),(2,7),(3,7),(4,7),(5,7),(6,7),(7,7),(8,7),
			(1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0),(8,1),(8,2),(8,3),(8,4),(8,5),(8,6),(8,0)]
	
	for i in range(len(obstacles)):
		obstacles[i] = (obstacles[i][0]-start[0],obstacles[i][1]-start[1])
	goal = (goal[0]-start[0],goal[1]-start[1])
	start = (0,0)
	path = get_path_from_A_star(start, goal, obstacles)
	for i in range(len(path)):
		path[i] = (path[i][0],path[i][1])
	final = (path[len(path)-1][0],path[len(path)-1][1])
	path.insert(len(path),final)
	path.insert(len(path),final)
	print(path)
	return path

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]
    
    parents = {}
    past_cost = {}
    cost_between_nodes = 1
    est_total_cost = {}
    past_cost[start] = 0
    to_visit_nodes = [(past_cost[start],start)] #to visit = open -> big tuple
    visited_nodes = [] #visited = closed 
    goal_set = get_neighbors(goal)

    while to_visit_nodes != []: 
        current = to_visit_nodes.pop(0)[1] #trying to get just the normal tuple
        #print(current)
	visited_nodes.append(current) #visited nodes is normal tuple
        #print(visited_nodes)
	if current in goal_set:
            path = assemble_path(parents,current,start,goal)
            return path
        current_neighbors = get_neighbors(current)        
        for i in range(len(current_neighbors)):
	    #print(i)
            #print(current_neighbors[i])
	    #print(visited_nodes)
            if current_neighbors[i] in visited_nodes:
                continue
            if current_neighbors[i] in obstacles:
                continue
            tentative_past_cost = past_cost[current] + cost_between_nodes 
            #if the neighbor current past cost is less than the new past cost or the neighbor has not been assigned a past cost yet
            if (current_neighbors[i] not in past_cost) or (tentative_past_cost < past_cost[current_neighbors[i]]):
                past_cost[current_neighbors[i]] = tentative_past_cost
                parents[current_neighbors[i]] = current
                est_total_cost[current_neighbors[i]] = past_cost[current_neighbors[i]] + heuristic_cost_to_go(current_neighbors[i],goal)
                to_visit_nodes.append((est_total_cost,current_neighbors[i]))
                to_visit_nodes.sort(key = lambda x: x[0]) #sort nodes to visit based on which ones have the lowestestimated total cost
    path = [] #indicates failure
    return path

