#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveRobot:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_robot_node', anonymous=True)

        # Publisher to send velocity commands to the TurtleBot
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Subscriber to listen for movement commands
        rospy.Subscriber('move_command', String, self.command_callback)

        # Subscribe to Odometry topics
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_position = None 
        self.initial_position = None

        rospy.loginfo("Move Robot Node started. Waiting for movement commands on 'move_command' topic.")
        rospy.spin()

    def odom_callback(self, data): 
        # Callback function that process the odometry data 
        if self.current_position is None: 
            self.initial_position = data.pose.pose.position 
            self.current_position = self.initial_position 
        else: 
            self.current_position = data.pose.pose.position

    def command_callback(self, msg):
        command_str = msg.data
        try:
            # Expecting a command in the format: "<distance> <direction>"
            # For example, "5 0" means move for 5 meters in direction 0.
            parts = command_str.split()
            if len(parts) != 2:
                rospy.logerr("Command should consist of two numbers: <distance> <direction>")
                return

            distance = float(parts[0])
            direction = int(parts[1])
        except Exception as e:
            rospy.logerr("Error parsing command: %s", e)
            return

        rospy.loginfo("Received command: distance = %s meters, direction = %s", distance, direction)

        # Create a Twist message
        twist = Twist()

        # Define movements based on direction:
        # 0 = forward, 1 = backward, 2 = left turn, 3 = right turn
        if direction == 0:
            twist.linear.x = 1.0   # Move forward
        elif direction == 1:
            twist.linear.x = -1.0  # Move backward
        elif direction == 2:
            twist.angular.z = 0.5  # Turn left
        elif direction == 3:
            twist.angular.z = -0.5 # Turn right
        else:
            rospy.logerr("Invalid direction. Please use 0 (forward), 1 (backward), 2 (left), or 3 (right).")
            return

        self.initial_position = self.current_position 
        distance_travelled = 0

        # Publish the twist message repeatedly for the specified direction
        while distance_travelled < distance and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            dx = self.current_position.x - self.initial_position.x 
            dy = self.current_position.y - self.initial_position.y
            distance_travelled = math.sqrt(dx**2 + dy**2)

            

        # Stop the robot after moving
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        MoveRobot()
    except rospy.ROSInterruptException:
        pass
