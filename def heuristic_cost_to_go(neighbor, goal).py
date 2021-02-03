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
	obstacles = [(0,0), (1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0),(9,0),(10,0),(11,0),(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(1,6),(2,6),(3,6),(4,6),(5,6),(6,6),(7,6),(8,6),(9,6),(10,6),(11,6),(11,1),(11,2),(11,3),(11,4),(11,5),(11,6),(4,3),(4,2),(4,1),(7,3),(7,4),(7,5)]
	
	for i in range(len(obstacles)):
		obstacles[i] = (obstacles[i][0]-start[0],obstacles[i][1]-start[1])
	goal = (goal[0]-start[0],goal[1]-start[1])
	start = (0,0)
	path = get_path_from_A_star(start, goal, obstacles)
	for i in range(len(path)):
		path[i] = (.5*path[i][0],.5*path[i][1])
	final = (path[len(path)-1][0]+.5,path[len(path)-1][1])
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

#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from motion_planning import get_waypoint_path


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)


        #global variables for control
        self.count = 0 # define variable to count how many way points have happened
        self.prior_velocity = np.matrix([0,0]) # define variable for previous value of velocity
        self.prev_xvel = 0
        self.prev_yvel = 0
        self.error1 = 0
        self.error0 = 0

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel = Twist()
        self.T = 1

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
	start = (2,3) 
	goal = (9,4)
        waypoints = get_waypoint_path(start,goal)
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        timeStep = .1 #time step based on the frequency
        t = np.arange(0, self.T+timeStep, timeStep) #create t array for each time instance
        x_start = self.pose.x
        y_start = self.pose.y
        x_end = current_waypoint[0]
        y_end = current_waypoint[1]
        avg_velocity = .1


        next_vector = [ next_waypoint[0] - self.pose.x, next_waypoint[1] - self.pose.y]
        angle2= atan2(next_vector[1], next_vector[0])
    
        
        #now we determine the coefficients for x and y 
        a_x = self.polynomial_time_scaling_3rd_order(x_start, self.prev_xvel, x_end, avg_velocity*cos(angle2),self.T)
        a_y = self.polynomial_time_scaling_3rd_order(y_start, self.prev_yvel, y_end, avg_velocity*sin(angle2), self.T)

        self.prev_xvel = avg_velocity*cos(angle2)
        self.prev_yvel = avg_velocity*sin(angle2)
  
        
        kp=5
        kd=.1
        x_t = np.poly1d([a_x[0][0],a_x[1][0],a_x[2][0],a_x[3][0]])
        y_t = np.poly1d([a_y[0][0],a_y[1][0],a_y[2][0],a_y[3][0]])
        #This for loop is meant to iterate for the number of elements in t which was declared above
        for i in t:
            x_velocity = np.polyder(x_t, 1)
            y_velocity = np.polyder(y_t, 1)
            self.vel.linear.x = sqrt(x_velocity(i)**2 + y_velocity(i)**2)

            theta = atan2(y_velocity(i), x_velocity(i))
            
            self.vel.angular.z = self.pd_controller(kp, kd, theta)
        
            #if((sqrt(self.pose.x**2+self.pose.y**2) < .2) & self.count != 0): #slow down the robot when it approaches home again
             #   self.vel.angular.z /= 2
             #   self.vel.linear.x /= 2

            self.vel_pub.publish(self.vel) #publish derived velocities to robot
            self.rate.sleep()
        self.count += 1
        print self.count
        
        pass

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        T_mat = np.array([[0,0,0,1],
                	[self.T**3,self.T**2,self.T,1],
                	[0,0,1,0],
                	[3*self.T**2,2*self.T,1,0]])
        x = np.array([[p_start],[p_end],[v_start],[v_end]])
        a = np.linalg.solve(T_mat,x)

        return a

    def pd_controller(self, kp, kd, theta):
        
        #rewrite the PD controller 
        self.error1 = self.error0
        self.error0 = (theta - self.pose.theta)
        if self.error0 > pi:
            self.error0 = self.error0 - 2*pi
        elif self.error0 < -pi:
            self.error0 = self.error0 + 2*pi
            
        #implment equation in lab3 , P and D parameters
        omega = kp*(self.error0) + kd*(self.error0 - self.error1)

        return omega
    
    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))
    
class Controller:
    def __init__(self, P=0.0, D=0.0, Derivator=0):
        self.Kp = P
        self.Kd = D
        self.Derivator = Derivator
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.set_point == pi:
            self.error = self.set_point - abs(current_value)
        elif abs(self.error) > pi:
            self.error = 2*pi - self.error
            P_value = self.Kp*self.error
            D_value = self.Kd*(self.error-self.Derivator)
            self.Derivator = self.error
            return P_value + D_value

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0

    def setPD(self, set_P=0.0, set_D=0.0):
        self.Kp = set_P
        self.Kd = set_D


if __name__ == '__main__':
    whatever = Turtlebot()