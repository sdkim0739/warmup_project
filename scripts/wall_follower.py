#!/usr/bin/env python3
''' This script makes Turtlebot3 follow walls from a fixed distance '''
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

# Distance from wall from which robot will drive along wall
dist_from_wall = 0.5

# Turtlebot3 follows along walls at a fixed distance, turning at corners
class WallFollower(object):
    # Initialize the node, publisher, subscriber, and Twist message instance
    def __init__(self):
        # Node for wall follower
        rospy.init_node('wall_follower')

        # Publisher to /cmd_vel
        self.cv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocities = Twist()

        # Subscriber to /scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

    # Callback function for subscriber
    def process_scan(self, data):
        # front is the front of the robot, i.e. the 10 degree range in front of the robot, and 
        front = min(min(data.ranges[355:360]), min(data.ranges[0:5]))
        # right is the right side of the robot, around 90 degrees +/- 5.
        right = min(data.ranges[90:92])
        # front_right is the angle range from 6 to 89 degrees relative to the robot
        front_right = min(data.ranges[6:90])

        # If the robot is not the specified distance from the wall, move it towards the wall
        if front > dist_from_wall and front_right > dist_from_wall:
            self.velocities.linear.x = 0.3
            self.velocities.angular.z = 0
        else: # If the robot is within the specified distance from the wall, do the following
            # If the robot's right side is not the specified distance from the wall (i.e. robot's right side isn't parallel to the wall),
            # rotate it to its left.
            if right > dist_from_wall:
                self.velocities.linear.x = 0
                self.velocities.angular.z = 0.3
            else: # Once the right side is the specified distance away from the wall, stop rotating
                self.velocities.linear.x = 0.3
                self.velocities.angular.z = 0

        # Update the velocities
        self.cv_pub.publish(self.velocities)

    # Runs the Turtlebot's movement until a shutdown signal is given
    def run(self):
        rospy.spin()

# Runs the program as a whole
if __name__ == '__main__':
    node = WallFollower()
    node.run()