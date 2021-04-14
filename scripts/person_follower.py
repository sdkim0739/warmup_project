#!/usr/bin/env python3
''' This script makes Turtlebot3 follow around a person / object '''
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

# Distance from object at which Turtlebot stops
distance = 0.5

# Turtlebot3 follows a person / object
class PersonFollower(object):
    # Initialize the node, publisher, subscriber, and a Twist message instance
    def __init__(self):
        rospy.init_node('person_follower')

        # Publish to /cmd_vel
        self.cv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocities = Twist()

        # Subscribe to /scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

    # Callback function used in subscriber
    def process_scan(self, data):
        distance_to_obj = min(data.ranges) # Minimum distance from robot to object
        
        # Angle, in degrees and radians, at which distance_to_obj is calculated
        angle_to_obj_deg = data.ranges.index(distance_to_obj)
        angle_to_obj_rad = angle_to_obj_deg * 3.14 / 180

        # If there's no object in the map, return
        if distance_to_obj == float("inf"):
            return

        # If the 10 degree angle range in front of the robot does not contain the object, rotate the robot until it does
        if not (min(data.ranges[0:10]) >= distance_to_obj-0.05 and min(data.ranges[0:10]) <= distance_to_obj+0.05):
            # Turns the robot left / right depending on which side of the robot the object is on
            if angle_to_obj_deg > 180: 
                self.velocities.angular.z = -0.3
            else:
                self.velocities.angular.z = 0.3

            # Make sure the robot can't move forward while rotating
            self.velocities.linear.x = 0
        else: # Otherwise, the object is in that 10 degree angle range in front of the robot. 
            # The robot should just move forward towards it
            self.velocities.angular.z = 0 # Make sure the robot can't rotate while moving forward
            self.velocities.linear.x = 0.3

        self.cv_pub.publish(self.velocities)

        # If the robot is within the specified distance from the object to stop, stop moving forward
        if distance_to_obj <= distance: 
            self.velocities.linear.x = 0

        self.cv_pub.publish(self.velocities)

    # Runs the bot until a shutdown command is given
    def run(self):
        rospy.spin()

# Runs the program as a whole
if __name__ == "__main__":
    node = PersonFollower()
    node.run()