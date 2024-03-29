#!/usr/bin/env python

import rospy
import os

class CommandExecutor:
    def __init__(self):
        rospy.init_node('command_executor', anonymous=True)
        # Initialize any variables or services here

if __name__ == '__main__':
    command_executor = CommandExecutor()
    rospy.spin()


class CommandExecutor:
    def __init__(self):
        rospy.init_node('command_executor', anonymous=True)
        self.command_sub = rospy.Subscriber('/high_level_commands', String, self.command_callback)

    def command_callback(self, msg):
        # Handle the command message
        self.process_command(msg.data)

    # Add commands here
    def process_command(self, command):
        if command == 'start_navigation':
            os.system("roslaunch my_robot_navigation navigation.launch")
        elif command == 'stop_navigation':
            os.system("rosnode kill navigation_node")
