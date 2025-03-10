#!/usr/bin/env python

from openai import OpenAI

client = OpenAI(api_key="sk-REPLACE_ME")
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MoveRobotWithOpenAI:
    def __init__(self):
        rospy.init_node('move_robot_openai', anonymous=True)
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # OPENAI KEY (note: leaving blank purposefully on this repo)
        file_name = "command.txt"

        prompt_text = self.read_prompt_file(file_name)
        duration, direction = self.get_move_parameters_from_openai(prompt_text)

        if duration is not None and direction is not None:
            rospy.loginfo("Successfully parsed: duration: %s, direction: %s", duration, direction)
            self.move_robot(duration, direction)
        else:
            rospy.logerr("Could not parse, something went wrong.")

    def read_prompt_file(self, filename):
        try:
            with open(filename, 'r') as f:
                content = f.read()
            rospy.loginfo("Was able to read prompt from '%s'.", filename)
            return content
        except Exception as e:
            rospy.logerr("Could not read file '%s': %s", filename, e)
            return ""

    def get_move_parameters_from_openai(self, prompt_text):
        system_message = (
            "You are the brain of a turtlebot, and you are provided with a prompt that a human has given you"
            "The prompt consists of a length in time the robot should move and direction (where direction is one of: 0=forward, 1=backward, 2=left, 3=right). "
            "Return them in the format: <duration> <direction>."
        )

        user_message = f"User's command: {prompt_text}\n\n"

        try:
            response = client.chat.completions.create(model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_message}
            ],
            temperature=0.0)  # NOTE: temperature changes determinism level of output)
            content = response.choices[0].message.content.strip()
            rospy.loginfo("OpenAI raw response: %s", content)

            parts = content.split()
            if len(parts) == 2:
                duration = float(parts[0])
                direction = int(parts[1])
                return duration, direction
            else:
                rospy.logerr("Response not extracted correctly. Expected 'duration direction'.")
                return None, None

        except Exception as e:
            rospy.logerr("Error calling OpenAI API: %s", e)
            return None, None

    def move_robot(self, duration, direction):
        twist = Twist()

        if direction == 0:
            twist.linear.x = 0.2   # forward
        elif direction == 1:
            twist.linear.x = -0.2  # backward
        elif direction == 2:
            twist.angular.z = 0.5  # left
        elif direction == 3:
            twist.angular.z = -0.5 # right
        else:
            rospy.logerr("Invalid direction. Use 0=forward,1=backward,2=left,3=right.")
            return

        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)  # 10 Hz

        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        MoveRobotWithOpenAI()
    except rospy.ROSInterruptException:
        pass
