
#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D


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
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                      [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                      [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(14):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        timeStep = .1 #time step based on the frequency
        t = np.arange(0, self.T+timeStep, timeStep) #create t array for each time instance
        x_start = self.pose.x #initial x value for each segment
        y_start = self.pose.y #initial y value for each segment
        x_end = current_waypoint[0] #final x value for each segment
        y_end = current_waypoint[1] #final y value for each segment
        avg_velocity = .1 

        #find vector from current point to the next waypoint
        next_vector = [ next_waypoint[0] - self.pose.x, next_waypoint[1] - self.pose.y]
        #find angle of vector from current point to next point
        angle2= atan2(next_vector[1], next_vector[0])
        
        #now we determine the coefficients for x and y 
        # use self.prev_vel (final velocity of previous segment) as initial velocity and use x,y components
        # of angle2 above to determine direction and magnitude of v_end
        a_x = self.polynomial_time_scaling_3rd_order(x_start, self.prev_xvel, x_end, avg_velocity*cos(angle2),self.T)
        a_y = self.polynomial_time_scaling_3rd_order(y_start, self.prev_yvel, y_end, avg_velocity*sin(angle2), self.T)

        # save the current final velocity to be used as initial velocity on next segment
        self.prev_xvel = avg_velocity*cos(angle2)
        self.prev_yvel = avg_velocity*sin(angle2)
  
        #define PD control parameters
        kp=5
        kd=.1

        #obtain position polynomial from coefficients found with polynomial time scaling
        x_t = np.poly1d([a_x[0][0],a_x[1][0],a_x[2][0],a_x[3][0]])
        y_t = np.poly1d([a_y[0][0],a_y[1][0],a_y[2][0],a_y[3][0]])
        
        for i in t:
            #take derivative of position polynomials
            x_velocity = np.polyder(x_t, 1)
            y_velocity = np.polyder(y_t, 1)

            #set linear velocity to the combined magnitude of x/y velocities evaluated at the current time
            self.vel.linear.x = sqrt(x_velocity(i)**2 + y_velocity(i)**2)

            # desired angle is set to be the angle between our velocity vector and the fixed frame
            theta = atan2(y_velocity(i), x_velocity(i))
            # use PD controller to obtain necessary angular velocity
            self.vel.angular.z = self.pd_controller(kp, kd, theta)

            self.vel_pub.publish(self.vel) #publish derived velocities to robot
            self.rate.sleep()
        self.count += 1
        
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
    

if __name__ == '__main__':
    whatever = Turtlebot()