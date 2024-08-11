#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class ActionController(Node):

    def __init__(self):
        super().__init__('action_controller')
        self.state = 'standby'
        

        # Publisher for the current state
        self.state_publisher = self.create_publisher(String, 'action_controller_state', 10)

        # Subscriptions
        self.create_subscription(String, 'predefined_path_button', self.set_predefined_path_state, 10)
        self.create_subscription(String, 'command_start_mapping', self.set_map_state_with_command, 10)
        self.create_subscription(String, 'mapping_done', self.mapping_done, 10)
        self.create_subscription(String, 'system_state_request', self.handle_state_change, 10)
        

    def publish_state(self):
        # This method publishes the state when it changes
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Published the current system state: {msg.data}')
    
    def handle_state_change(self, msg):

        if (msg.data == 'joystick_control'):

            if (self.state == 'standby'):
                self.state = 'joystick_arm_control'
                self.publish_state()
                return

            elif (self.state == 'joystick_arm_control'):
                self.state = 'joystick_rail_control'
                self.publish_state()
                return
            else:
                self.get_logger(f'The joystick button press was ignored -> The system is in a state that cannot be changed: {self.state}')
                return

        elif (msg.data == 'joystick_arm_control'):

            if (self.state == 'joystick_rail_control'):
                self.state = 'joystick_arm_control'
                self.publish_state()
                return
            else:
                pass

        elif (msg.data == 'run_predefined_path'):

            if (self.state == 'standby'):
                self.state = msg.data
                self.publish_state()
                return
            else:
                self.get_logger().info(f'The run a predefined path button press was ignored -> The system is in a state that cannot be changed: {self.state}')
                return

        elif (msg.data == 'map'):
             
             if (self.state == 'standby'):
                 self.state = msg.data
                 self.publish_state()
                 return
             else:
                 self.get_logger().info(f'The map button press was ignored -> The system is in a state that cannot be changed: {self.state}')
                 return
        
        elif (msg.data == 'standby'):

            if (self.state == 'standby'):
                self.get_logger().info(f'The reset/"stop the active process" button press was ignored -> The system is already in the {self.state} state')
                return

            else:
                self.state = msg.data
                self.publish_state()
                return
        

    def set_predefined_path_state(self, msg): # TODO needs rework
        if msg.data == 'pressed':
            self.state = 'predefined_path'
            self.publish_state()
            self.get_logger().info('Setting state: Starting predefined path')

    def set_map_state_with_command(self, msg):
        if msg.data == 'start' and self.state == 'standby':
            self.state = 'map'
            self.publish_state()
            self.get_logger().info('state set: map')

    def mapping_done(self, msg):
        if msg.data == 'done':
            self.state = 'standby'
            self.publish_state()
            self.get_logger().info('Setting state: Mapping completed. Returning to standby state.')

def main(args=None):
    rclpy.init(args=args)
    action_controller = ActionController()
    try:
        rclpy.spin(action_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()