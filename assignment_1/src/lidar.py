#! /usr/bin/env python

""" Adapted from roguebasin.com for bresenham line calculation and
reference from https://github.com/ERC-BPGC/Trotbot/blob/8ff34049b9c81fa50d29493b5669140b0f75d0d5/navigation/scripts/mapping/lidar_to_grid.py
for xy(m) to ij(pixel) conversion 
"""
import math  # Python Math library used here for cos and sin functions
import rospy  # ROS Python library
# ROS LaserScan message for Lidar datatype
from sensor_msgs.msg import LaserScan
# ROS Odometry for Robot position and
from nav_msgs.msg import Odometry, OccupancyGrid
# ROS Map message for Grid Map visualisation
from geometry_msgs.msg import Pose  # ROS Position message to define map origin
# To convert quaternion to euler to get yaw data
from tf.transformations import euler_from_quaternion
import numpy as np  # Numpy library for array & Matrices


class lidar:
    def __init__(self):
        rospy.init_node('lidar') #naming the rosnode as lidar although it will be overwritten from launch file
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback) #Subscribing to /odom topic for pose data
        self.lidar = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1) #Subscribing to /scan topic for lidar data
        self.map_msg = OccupancyGrid()  #initialising occupancy grid ros message for publishing generated map
        self.map_msg.header.frame_id = 'odom' #publishing map in odom frame as it is the world frame
        self.map_msg.info.resolution = 0.2  #using resolution as 0.2, per pixel
        self.map_msg.info.width = 75        #using columns in map grid as 75
        self.map_msg.info.height = 75       #using rows in map grid as 75
        
        self.map_msg.data = [-1] * 75 * 75  #initialising the grid with -1 for unknown space
        self.pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True) #setting publisher for publishing generated map
        
        #waiting to receive a message before spinning the callbacks
        self.temp = rospy.wait_for_message('/odom', Odometry)
        self.temp = rospy.wait_for_message('/scan', LaserScan)
        rospy.spin() #Enabling the callback functions

    def bresenham(self, start, end):
        """
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        >>> points1 = bresenham((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """

        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        points = np.array(points)
        return points

#Callback function from /scan topic that reads the laser data, converts to obs frame, 
#then converts to world frame, populates the grid structure with obstacle data and publishes
#the map
    
    def lidar_callback(self, msg):
        #Creating angle array for each of the lidar data range in array
        self.ang = np.arange(msg.angle_min, msg.angle_max,(msg.angle_max - msg.angle_min)/len(msg.ranges))
        
        #Storing the ranges of lidar in a variable 
        scan = np.array(msg.ranges)
        scan = np.where(scan == np.inf, 0, scan)    #Replacing Infinity values with 0
        scan = np.where(scan == -np.inf, 0, scan)   #Replacing Infinity values with 0


        ox = np.cos(self.ang) * scan                #Getting the obstacle data in x axis obstacle frame
        oy = np.sin(self.ang) * scan                #Getting the obstacle data in y axis obstacle frame

        #Translation and Rotation matrix is calculated from odometry callback to convert values from obstacle frame to world frame
        rot_x = (self.rot_cos * ox) - (self.rot_sin*oy) 
        rot_y = (self.rot_sin * ox) + (self.rot_cos*oy)

        #Finally we have the obstacle data in world frame
        world_x = self.trans_x + rot_x
        world_y = self.trans_y + rot_y

        # Initialize an empty gridmap with 75 x 75 cells across 15m
        self.pmap = np.zeros((int(round(15 / 0.2)), (int(round(15 / 0.2)))))
        # centx,centy centre of gridmap is calculated to identify map's origin
        centx = int(round(75/2) - 1)
        centy = int(round(75/2) - 1)
        
        INC = max(1, int(0.5 / (2 * 0.2)))  # Obstacle Inflation purpose

# For loop to iterate over the world values and inflate area around the obstacle with prob 1
        for (x,y) in zip(world_x , world_y):
            try:
                ix = centx + int(round(x/0.2))
                iy = centy + int(round(y/0.2))
                self.pmap[iy-INC:iy+INC+1 , ix-INC:ix+INC+1] = 1
            except:
                pass
        
# Find the robot's location with respect to the grid frame
        r_ix = centx + int(round(self.robot_pose.x/0.2))
        r_iy = centy + int(round(self.robot_pose.y/0.2))
        
# For loop to iterate over the world values fill the area having obstacles with prob 1
        for (x, y) in zip(world_x, world_y):
             
            try:
                ix = centx + int(round(x/0.2))
                iy = centy + int(round(y/0.2))
                self.pmap[iy, ix] = 1
                #print(self.pmap[iy,ix])
                #bresenham calculation to identify free space. Commented out for better visualisation
                """free_area = self.bresenham((centx, centy), (ix, iy))
                for fa in free_area:
                    self.pmap[fa[1]-1][fa[0]-1] = 0"""

            except:
                pass
        self.pmap[r_iy, r_ix] = 0   #Fill robot's position as free space in grid

        #Fill area around robot's position as free space in grid
        self.pmap[r_iy-INC:r_iy+INC+1 , r_ix-INC:r_ix+INC+1] = 0 

        self.pmap = self.pmap.astype(int)   #Converting the datatype to int 
        self.present_pos = Pose()           #Initialising Pose message to define origin in map
        self.present_pos.position.x -= (15/2)   #X Origin as -7.5 in m
        self.present_pos.position.y -= (15/2)   #Y Origin as -7.5 in m
        self.map_msg.header.frame_id = "odom"   
        self.map_msg.info.resolution = 0.2  
        self.map_msg.info.width = int(round(15 / 0.2))
        self.map_msg.info.height = int(round(15 / 0.2))
        self.map_msg.info.origin = self.present_pos
        self.map_msg.data = list((self.pmap.flatten()*100)) #reshape the grid to a 1D array and store as list
        self.pub.publish(self.map_msg)  #Publish the map message

#Callback function from /odom topic that stores the position and orientation data. Orientation is then converted 
#to euler using the ros function
 
    def odom_callback(self, msg):
        self.odom_var = msg.pose.pose.orientation
        self.robot_pose = msg.pose.pose.position    #Save the robot's current position on world frame 
        self.orientation_list = [self.odom_var.x, self.odom_var.y, self.odom_var.z, self.odom_var.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list) #Convert quaternion to Euler

        # The following logic was necessary to align the map with the laser readings. ROS Odometry
        # represents right direction in negatives and left in positive as opposed to transformation
        # frame.
        if self.yaw > 0:
            self.yaw = self.yaw - 3.141592653589793238
        elif self.yaw < 0:
            self.yaw = self.yaw + 3.141592653589793238
        elif self.yaw == 0:
            self.yaw = self.yaw

        #The following variable forms the translation matrix
        self.trans_x = msg.pose.pose.position.x
        self.trans_y = msg.pose.pose.position.y

        #The following variables together form the rotation matrix
        self.rot_cos = math.cos(self.yaw)
        self.rot_sin = math.sin(self.yaw)
       


if __name__ == '__main__':
    l = lidar()
