#! /usr/bin/env python

""" Adapted from Constructsim courses of path planning through potential field
    Velocity generation adapted from https://github.com/verlab/hero_common
"""

import math  # Python Math library used here for cos and sin functions
import rospy  # ROS Python library
# ROS LaserScan message for Lidar datatype
from sensor_msgs.msg import LaserScan
# ROS Odometry for Robot position and
from nav_msgs.msg import Odometry, OccupancyGrid
# ROS Map message for Grid Map visualisation
# ROS Position message to define map origin
from geometry_msgs.msg import Pose, PoseStamped, Twist
# To convert quaternion to euler to get yaw data
from tf.transformations import euler_from_quaternion
import numpy as np  # Numpy library for array & Matrices

import threading    #Threading library to run separate loops


class Field:
    def __init__(self):
        
        rospy.init_node('Potential_Field') #Initialise ros node name
        #The following variable pulls the goal x and y from param
        self.goal = (rospy.get_param("/potential_field/goal_x"),rospy.get_param("/potential_field/goal_y"))
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback) #Odom topic subscription
        self.map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)   #Subscribe to /map for generating potential field
        
        #Subscribe to goal topic to do update goal positions
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        #Lidar topic subscription
        self.lidar = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        #Occupancy grid topic for publishing attraction field
        self.field_att_pub = rospy.Publisher('/att_field_map', OccupancyGrid, queue_size=1)

        #Occupancy grid topic for publishing repulsion field
        self.field_rep_pub = rospy.Publisher('/rep_field_map', OccupancyGrid, queue_size=1)

        #Velocity topic for publsihing velocity to robot
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #initialise robots current position. Will be updated in odometry callback
        self.robot = (0, 0)
        self.field_map = OccupancyGrid()        #initialise map message for fields publishing
        self.field_map.info.width = 75
        self.field_map.info.height = 75
        self.field_map.info.resolution = 0.2
        self.field_map.info.origin.position.x -= (15/2)
        self.field_map.info.origin.position.y -= (15/2)
        self.field_map.header.frame_id = "odom"
        self.force_value = 0    #Initialising the overall force value variable
        self.temp = rospy.wait_for_message('/odom', Odometry)   #Waiting for the first message
        self.temp = rospy.wait_for_message('/scan', LaserScan)  #Waiting for the first message
        self.rate = rospy.Rate(20)                              #Publishing rate for velocity message
        rospy.on_shutdown(self.myhook)                          #Making robot stop when exiting code
        rospy.loginfo('..')
        t = threading.Thread(target=self.run_robot)             #Initialise threading for run_robot loop
        t.daemon = True                                         #Initialise threading for run_robot loop
        t.start()                                               #Start threading
        rospy.spin()                                            #Enable callbacks

#The following callback calculates the repulsive force based on the obstacles stored in the grid 
#received from /map topic. For visualisation only

    def repulsion(self, position, obstacle, beta, q_max):

        dx = obstacle[0] - position[0]
        dy = obstacle[1] - position[1]
        distance = (dx ** 2 + dy ** 2)**0.5
        if (distance <= 0):
            distance = 1e-06

        return beta * ((1.0 / q_max) - (1.0 / distance)) * (1.0 / distance * distance)

#The following callback calculates the attraction force based on the goal and currebt cell. For visualisation only
    def conical_attractive_force(self, current_cell, goal_cell, K=10.0):
        """
        Calculates the linear attractive force for one grid cell with respect to the target
        current_cell: a list containing x and y values of one map grid cell
        goal_cell: a list containing x and y values of the target grid cell
        K: potential attractive constant
        returns: linear attractive force scaled by the potential attractive constant"""
        dx = goal_cell[0] - current_cell[0]
        dy = goal_cell[1] - current_cell[1]
        distance = (dx ** 2 + dy ** 2)**0.5
        return K * distance

#Lidar callback to store lidar data
    def lidar_callback(self, msg):
        self.lidar_msg = msg

#Goal callback  to store position of goal x, y 

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

#Odom callback to get robot current position and orientation

    def odom_callback(self, msg):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_var = msg.pose.pose.orientation
        self.orientation_list = [self.odom_var.x, self.odom_var.y, self.odom_var.z, self.odom_var.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)

        # The following logic was necessary to align the map with the laser readings. ROS Odometry
        # represents right direction in negatives and left in positive as opposed to transformation
        # frame.
        self.yaw_2 = self.yaw   #Storing raw yaw for future 
        if self.yaw > 0:
            self.yaw = self.yaw - 3.141592653589793238
        elif self.yaw < 0:
            self.yaw = self.yaw + 3.141592653589793238
        elif self.yaw == 0:
            self.yaw = self.yaw


#The following callback fuction receives the map and converts into 2d grid for potential field generation

    def map_callback(self, msg):

        force_repulsion = 0             #Initialise repulsion force
        force_attraction = 0            #Initialise attraction force
        centx = int(round(75/2) - 1)    #origin for xy to ij conversion
        centy = int(round(75/2) - 1)    #origin for xy to ij conversion
        ix = centx + int(round(self.goal[0]/0.2))   #convert goal xy to grid ij
        iy = centy + int(round(self.goal[1]/0.2))   #convert goal xy to grid ij
        robot_ix = centx + int(round(self.robot[0]/0.2))    #convert robot xy to grid ij
        robot_iy = centy + int(round(self.robot[1]/0.2))    #convert robot xy to grid ij
        self.robot_xy = (robot_ix, robot_iy)            #store robot position as array
        self.goal_xy = (ix, iy)                         #store goal position as array
        self.grid_attraction = np.array((msg.data))     #convert msg data into an numpy array
        self.grid_repulsion = np.ones((msg.info.height * msg.info.width)) * -1  #Initialise grid repulsion array
        self.grid_attraction.reshape(msg.info.height, msg.info.width)       #Reshape the matrices to 2d array
        self.grid_repulsion.reshape(msg.info.height, msg.info.width)        #Reshape the matrices to 2d array

        #For loop to iterate over the grid to calculate attractive force between goal cell and each cell, 
        #repulsive force between obstacle cell and  each cell if the distance is less than 0.2m(1px)

        for row in range(msg.info.height):
            for col in range(msg.info.width):
                force_attraction = self.conical_attractive_force([row, col], self.goal_xy)
                if force_attraction < -1:       #Checking if values are between -1 to 100
                    force_attraction = -1
                elif force_attraction > 100:
                    force_attraction = 100

                if self.grid_attraction[row+msg.info.width*col] == 100: #calculating repulsive force for cells having obstacle 
                    force_repulsion = self.repulsion(self.robot_xy, [row, col], 0.5, 1) #rep_gain in 0.5 
                else:
                    force_repulsion = -1            #if no obstacle store as unknown space for visualisation purpose
                self.grid_attraction[row +msg.info.width*col] = force_attraction
                self.grid_repulsion[row + msg.info.width*col] = force_repulsion

        self.grid_repulsion = self.grid_repulsion.astype(int)   #Convert datatype to int for publishing

        #self.field_map = msg
        self.field_map.data = list((self.grid_attraction.flatten()))    #change grid to 1D array amd list
        self.field_att_pub.publish(self.field_map)                      #Publish attraction grid
        self.field_map.data = list((self.grid_repulsion.flatten()))     #change grid to 1D array amd list
        self.field_rep_pub.publish(self.field_map)                      #Publish repulsion grid

#Function to execute when code is exiting. Makes the robot stop

    def myhook(self):
        vel = Twist()
        self.vel_pub.publish(vel)

#Function that runs in a thread to run the robot towards the goal through attraction field and away from
#obstacles through repulsion

    def run_robot(self):
        while True:

            target_dist = math.hypot(self.goal[0]-self.robot[0], self.goal[1]-self.robot[1])    #Calculate distance between robot and goal

            while target_dist > 0.2:    #Threshold value for reaching goal

                dx = self.goal[0] - self.robot[0]   #calculate x distance 
                dy = self.goal[1] - self.robot[1]   #calculate y distance 

                K = 1.2                             #Attraction gain
                B = 0.4                             #Repulsion gain
                distance = (dx ** 2 + dy ** 2)**0.5   #Distance for potential  
                theta = math.atan2(dy, dx)             #Heading angle calculation
                yaw = self.yaw_2                        #Robot curent yaw
                fatt_x = K * math.cos(theta - yaw) * distance   #x force in attraction
                fatt_y = K * math.sin(theta - yaw) * distance   #y force in attraction
                frep_x = 0                                      #Initialise repulsion force
                frep_y = 0                                         #Initialise repulsion force
                rep_dist = 0.2                                     #q_max hreshold for calculating repulsion
                #For loop for finding repulsion force through lidar data
                for i in range(len(self.lidar_msg.ranges)):
                    r = self.lidar_msg.ranges[i]
                    if r <= rep_dist:
                        alpha = i * self.lidar_msg.angle_increment + self.lidar_msg.angle_min
                        frep_x += B * \
                            (1.0/r - 1.0/rep_dist) * \
                            math.cos(alpha)/(r**2)
                        frep_y += B * \
                            (1.0/r - 1.0/rep_dist) * \
                            math.sin(alpha)/(r**2)

                #combine attarction and repulsion
                fx = fatt_x - frep_x
                fy = fatt_y - frep_y
                fr = math.sqrt(fx**2 + fy**2)

                #Limit the velocity to max 0.5m/s
                x_vel = min(fr, 0.5)
                z_vel = (math.atan2(fy, fx))    #find the required turning angle for travelling to goal
                if z_vel > math.pi:             #Limit max ang velicity between -3.14 to +3.14
                    z_vel = - 2 * math.pi + z_vel
                elif z_vel < -math.pi:
                    z_vel = 2 * math.pi + z_vel

                velocity = Twist()              #Velocity message initialisation
                velocity.linear.x = x_vel       
                velocity.angular.z = z_vel
                target_dist = math.hypot(self.goal[0]-self.robot[0], self.goal[1]-self.robot[1])
                self.vel_pub.publish(velocity)  #publish velocity
            print("stopping")
            velocity = Twist()
            self.vel_pub.publish(velocity)      #publish stop velcoity after reaching goal
            self.rate.sleep()


if __name__ == '__main__':
    f = Field()
