#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MoveRobot:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('/mobile_base/commands/velocity', anonymous=True)

        # Publisher to send velocity commands to the TurtleBot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to listen for movement commands
        rospy.Subscriber('move_command', String, self.command_callback)

        rospy.loginfo("Move Robot Node started. Waiting for movement commands on 'move_command' topic.")
        rospy.spin()

    def command_callback(self, msg):
        command_str = msg.data
        try:
            # Expecting a command in the format: "<duration> <direction>"
            # For example, "5 0" means move for 5 seconds in direction 0.
            parts = command_str.split()
            if len(parts) != 2:
                rospy.logerr("Command should consist of two numbers: <duration> <direction>")
                return

            duration = float(parts[0])
            direction = int(parts[1])
        except Exception as e:
            rospy.logerr("Error parsing command: %s", e)
            return

        rospy.loginfo("Received command: duration = %s seconds, direction = %s", duration, direction)

        # Create a Twist message
        twist = Twist()

        # Define movements based on direction:
        # 0 = forward, 1 = backward, 2 = left turn, 3 = right turn
        if direction == 0:
            twist.linear.x = 0.2   # Move forward
        elif direction == 1:
            twist.linear.x = -0.2  # Move backward
        elif direction == 2:
            twist.angular.z = 0.5  # Turn left
        elif direction == 3:
            twist.angular.z = -0.5 # Turn right
        else:
            rospy.logerr("Invalid direction. Please use 0 (forward), 1 (backward), 2 (left), or 3 (right).")
            return

        # Publish the twist message repeatedly for the specified duration
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)  # 10 Hz
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop the robot after moving
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        MoveRobot()
    except rospy.ROSInterruptException:
        pass
