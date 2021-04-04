#!/usr/bin/env python3
''' This script makes Turtlebot3 drive in a square '''
import rospy
from geometry_msgs.msg import Twist, Vector3

# Class DriveSquare handles code for driving in a square
class DriveSquare(object):
    # General init function - establish publisher and velocities object
    def __init__(self):
        rospy.init_node('cmd_vel_pub')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocities = Twist()

    # Moves the bot forward in the x-direction
    def move_forward(self):
        self.velocities.linear.x = 0.1
        self.pub.publish(self.velocities)

    # Rotates the bot by 90 degrees (CCW)
    def rotate(self):
        # Defines variables to keep track of angle rotated, 90 degrees in radians, and time when rotation began
        current_angle = 0
        ninety_deg = 90 * 3.14 / 180
        previous_time = rospy.Time.now().to_sec()

        # Sets angular velocity only so robot rotates around z-axis
        self.velocities.linear.x = 0
        self.velocities.angular.z = 0.5

        # Rate at which angles are recorded
        r = rospy.Rate(5)
    
        # Until the angle of rotation is 90 degrees, keep increasing the angle of rotation
        while current_angle < ninety_deg:    
            self.pub.publish(self.velocities)
            current_time = rospy.Time.now().to_sec()
            current_angle = self.velocities.angular.z * (current_time - previous_time)
            r.sleep()

        # Stop the rotation after 90 degrees is reached
        self.velocities.angular.z = 0
        self.pub.publish(self.velocities)

    # Runs the forward movement and rotation in a loop to create a square path of movement
    def run(self):
        while not rospy.is_shutdown():
            self.move_forward()
            rospy.sleep(10)
            self.rotate()

''' Runs the program as a whole '''
if __name__ == '__main__':
    node = DriveSquare()
    node.run()