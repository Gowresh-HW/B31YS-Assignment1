#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# from services import obs_avoid, obs_avoid_response


speedpublisher = None


def callback(msg):
    # Retrieves LiDAR values between 290 to 336 (On the Left of the ROSBot)
    # for i in range(290, 336):
    rangeRight = min(min(msg.ranges[290:336]), 5)
    # Retrieves LiDAR values between 337 to 383 (In Front of the ROSBot)
    # for i in range(337, 383):
    rangeFront = min(min(msg.ranges[0:60]), 5)
    # Retrieves LiDAR values between 384 to 425 (On the Right of the ROSBot)
    # for i in range(384, 425):
    rangeLeft = min(min(msg.ranges[680:719]), 5)
    # Lidar information Stored in an Array Variable
    range_list = [rangeLeft, rangeFront, rangeRight]
    # Speed Control using Twist()
    speedcontrol = Twist()

    if (range_list[0] > safe and range_list[1] > safe and range_list[2] > safe):
        print("Going forward")
        speedcontrol.linear.x = velocity_v
        speedcontrol.angular.z = 0
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] > safe and range_list[1] > safe and range_list[2] < safe):
        print("Turning right")
        speedcontrol.linear.x = 0
        speedcontrol.angular.z = -velocity_omega
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] > safe and range_list[1] < safe and range_list[2] > safe):
        print("Turning left")
        speedcontrol.linear.x = 0
        speedcontrol.angular.z = velocity_omega
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] < safe and range_list[1] > safe and range_list[2] > safe):
        print("Going diagonally front left")
        speedcontrol.linear.x = velocity_v
        speedcontrol.angular.z = velocity_omega
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] > safe and range_list[1] < safe and range_list[2] < safe):
        print("Going diagonally back left")
        speedcontrol.linear.x = -velocity_v
        speedcontrol.angular.z = -velocity_omega
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] < safe and range_list[1] < safe and range_list[2] > safe):
        print("Turning left")
        speedcontrol.linear.x = 0
        speedcontrol.angular.z = velocity_omega
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] < safe and range_list[1] > safe and range_list[2] < safe):
        print("Going forward")
        speedcontrol.linear.x = velocity_v
        speedcontrol.angular.z = 0
        speedpublisher.publish(speedcontrol)
    elif (range_list[0] < safe and range_list[1] < safe and range_list[2] < safe):
        print("Turning right")
        speedcontrol.linear.x = 0
        speedcontrol.angular.z = -velocity_omega
        speedpublisher.publish(speedcontrol)
    else:
        rospy.loginfo(range_list)


def main():
    global speedpublisher
    rospy.init_node('scanval')
    speedpublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    # Initiating Constant Linear Velocity in x axis 'V'
    velocity_v = 0.25
    # Initiating Constant Angular Velocity in z axis 'W' (Omega)
    velocity_omega = 0.15
    # Initiating Constant Safe Distnace from Obstacles
    safe = 1.5
    main()