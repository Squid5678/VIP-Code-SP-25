#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from openai import OpenAI

client = OpenAI(api_key="sk-REPLACE_ME")

class MoveRobotCombined:
    def __init__(self):
        rospy.init_node('move_robot_combined', anonymous=True)

        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_position = None
        self.initial_position = None

        prompt_text = self.read_prompt_file("command.txt")
        distance, direction = self.get_move_parameters_from_openai(prompt_text)
        if distance is not None and direction is not None:
            rospy.loginfo("Parsed command: value = %s, direction = %s", distance, direction)
            self.move_robot(distance, direction)
        else:
            rospy.logerr("Failed to get valid movement parameters from OpenAI.")

    def odom_callback(self, data):
        if self.current_position is None:
            self.current_position = data.pose.pose.position
            self.initial_position = self.current_position
        else:
            self.current_position = data.pose.pose.position

    def read_prompt_file(self, filename):
        try:
            with open(filename, 'r') as f:
                content = f.read()
            rospy.loginfo("Successfully read command from '%s'.", filename)
            return content
        except Exception as e:
            rospy.logerr("Error reading file '%s': %s", filename, e)
            return ""

    def get_move_parameters_from_openai(self, prompt_text):
        system_message = (
            "You are the brain of a turtlebot. You are provided with a prompt "
            "that contains a distance (in meters for linear movement or as an angle for rotation) "
            "and a direction (where 0=forward, 1=backward, 2=left, or 3=right). "
            "Return the values in the format: <value> <direction>."
        )
        user_message = "User's command: " + prompt_text + "\n\n"

        try:
            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.0  # lower temperature for determinism
            )
            content = response.choices[0].message.content.strip()
            rospy.loginfo("OpenAI raw response: %s", content)
            parts = content.split()
            if len(parts) == 2:
                value = float(parts[0])
                direction = int(parts[1])
                return value, direction
            else:
                rospy.logerr("Unexpected response format from OpenAI: %s", content)
                return None, None
        except Exception as e:
            rospy.logerr("Error calling OpenAI API: %s", e)
            return None, None

    def move_robot(self, value, direction):
        # For linear movement (forward or backward)
        if direction == 0 or direction == 1:
            # Wait until an odom update is received before starting
            while self.current_position is None and not rospy.is_shutdown():
                rospy.sleep(0.1)
            self.initial_position = self.current_position

            twist = Twist()
            twist.linear.x = 0.2 if direction == 0 else -0.2

            distance_travelled = 0.0
            rate = rospy.Rate(10)  # 10Hz loop rate for publishing commands

            while distance_travelled < value and not rospy.is_shutdown():
                self.cmd_pub.publish(twist)
                # Calculate Euclidean distance from the initial position
                dx = self.current_position.x - self.initial_position.x
                dy = self.current_position.y - self.initial_position.y
                distance_travelled = math.sqrt(dx**2 + dy**2)
                rate.sleep()

            # Stop the robot after the required distance is reached
            self.cmd_pub.publish(Twist())
            rospy.loginfo("Linear movement complete. Distance travelled: %.2f meters", distance_travelled)

        # For rotation (turning left or right)
        elif direction == 2 or direction == 3:
            twist = Twist()
            # Set angular speed (in radians per second)
            if direction == 2:
                twist.angular.z = 1.57  # turning left
            else:
                twist.angular.z = -1.57  # turning right

            time_needed = abs(value) / abs(twist.angular.z)
            rospy.loginfo("Rotating for %.2f seconds to achieve an angle of %.2f radians", time_needed, value)
            end_time = rospy.Time.now() + rospy.Duration(time_needed)
            rate = rospy.Rate(10)  # 10Hz loop rate

            while rospy.Time.now() < end_time and not rospy.is_shutdown():
                self.cmd_pub.publish(twist)
                rate.sleep()

            # Stop rotation
            self.cmd_pub.publish(Twist())
            rospy.loginfo("Rotation complete. Angle turned: %.2f radians", value)
        else:
            rospy.logerr("Invalid direction received. Use 0 (forward), 1 (backward), 2 (left), or 3 (right).")
            return

if __name__ == '__main__':
    try:
        MoveRobotCombined()
    except rospy.ROSInterruptException:
        pass
