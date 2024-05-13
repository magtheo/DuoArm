#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')
        self.state = 'standby'
        

        # Publisher for the current state
        self.state_publisher = self.create_publisher(String, 'system_state', 10)

        # Subscriptions
        self.create_subscription(String, 'system_state_request', self.handle_state_change, 10)
        

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Published the current system state: {msg.data}')
    
    def handle_state_change(self, msg):
        self.get_logger().info(f'Handle state change request {msg.data}')
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
                self.get_logger().info(f'The joystick button press was ignored -> The system is in a state that cannot be changed: {self.state}')
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

            if (self.state == 'joystick_arm_control'):
                self.state = msg.data
                self.publish_state()
                self.get_logger().info(f'The reset/"stop the active process" button press was ignored -> The system is in a state that cannot be changed: {self.state}')
                return

            else:
                self.get_logger().info(f'The reset/"stop the active process" button press was ignored -> The system is in a state that cannot be changed: {self.state}')
                return
        elif (msg.data == 'path_done' or msg.data == 'mapping_done'):
            self.state = 'standby'
            self.publish_state()
            return
            

def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
