import serial
from .lss import *
import RPi.GPIO as GPIO
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
import json
import glob
import numpy as np


class HardwareInterfaceController(Node):
    def __init__(self):
        super().__init__('hardware_interface_controller')

        self.current_system_state = 'standby'
        self.previous_system_state = None

        self.state_subscriber = self.create_subscription(String, 'system_state', self.check_state_callback, 10)
        self.send_map_button_press_publisher = self.create_publisher(String, 'map_button_pressed', 10)
        self.set_system_state_by_request_publisher = self.create_publisher(String, 'system_state_request', 10)

        """Attribute related to joystick button pressing:"""
        self.joystick_button_pressed = 0

        """Attribute related to reset/"stop the active process" button pressing:"""
        self.reset_button_pressed = 0

        """Attribute related to the "run a predefined path" button pressing:"""
        self.run_predefined_path_button_pressed = 0

        """Attribute related to "mapping" button pressing:"""
        self.map_button_pressed = 0

        """Attributes and method related to communication between the Arduino MEGA, Raspberry PI, LSS adapter board and LSS motors:"""
        self.avail_usb_ports = None
        self.avail_serial_ports = None
        self.CST_LSS_Port = '/dev/lssMotorController'
        self.ser_obj_controller = serial.Serial('/dev/ttyACM1', 115200)
        self.CST_LSS_Baud = LSS_DefaultBaud
        initBus(self.CST_LSS_Port, self.CST_LSS_Baud)

        #self.ser_obj_controller.reset_input_buffer()
        self.wait_to_read_values_from_serial = False
       

        """Attributes related to the received analog values:"""
        self.x_analog_value = 0
        self.z_analog_value = 0
        self.diagonal_threshold = 255
        self.zero_value_deadzone = 100
        self.read_values_from_serial_clearance = True

        """Attributes and method related to the LSS motors and the rail system:"""
        self.diameter_of_wheel = 2.4 # [cm]
        self.circumference_of_wheel = np.pi * self.diameter_of_wheel # [cm]
        self.distance_to_travel = 10 # [cm]
        self.top_limit_lss1 = 0
        self.bottom_limit_lss1 = 0
        self.top_limit_lss0 = 0
        self.bottom_limit_lss0 = 0
        self.position_multiplier = 10
        self.lss0 = LSS(0)
        self.lss1 = LSS(1)
        self.lss2 = LSS(2)
        self.rail_position = None
        self.boundaries_and_last_rail_position_data = None
        self.boundaries_and_last_rail_position_data_updated = False
        self.set_boundaries_and_last_rail_position_data()

        """Attributes related to multiple button pressing cases:"""
        self.num_simultaneous_button_press_conditions = 10
        self.simultaneous_button_press_conditions = [0]*self.num_simultaneous_button_press_conditions

        """Attributes related to the standby state"""
        self.standby_logger_printed = False

    def set_boundaries_and_last_rail_position_data(self):
        try:
            with open('boundary_path_and_rail_position.json', 'r') as file:
                self.boundaries_and_last_rail_position_data = json.load(file)
                self.top_limit_lss0 = self.boundaries_and_last_rail_position_data['boundaries'][0]['top_lss0']*self.position_multiplier
                self.bottom_limit_lss0 = self.boundaries_and_last_rail_position_data['boundaries'][1]['bottom_lss0']*self.position_multiplier
                self.top_limit_lss1 = self.boundaries_and_last_rail_position_data['boundaries'][0]['top_lss1']*self.position_multiplier
                self.bottom_limit_lss1 = self.boundaries_and_last_rail_position_data['boundaries'][1]['bottom_lss1']*self.position_multiplier
                self.rail_position = self.boundaries_and_last_rail_position_data['last_rail_position'][0]['last_rail_pos']
                self.get_logger().info('Successfully set the boundaries for the LSS motors, and the last rail position in the rail_position variable')
                self.boundaries_and_last_rail_position_data_updated = True
        except FileNotFoundError:
                self.get_logger().error("File 'boundary_path_and_rail_position.json' not found.")
                return
    
    def save_last_rail_position(self):
        self.boundaries_and_last_rail_position_data['last_rail_position'][0]['last_rail_pos'] = self.rail_position
        with open('boundary_path_and_rail_position.json', 'w') as file:
            json.dump(self.boundaries_and_last_rail_position_data, file, indent=4)
        self.get_logger().info('Successfully stored the last rail position to the appropriate field in the JSON file')
           
    def lss1_beyond_bottom_limit(self):
        return int(self.lss1.getPosition()) < self.bottom_limit_lss1

    def lss1_beyond_top_limit(self):
        return int(self.lss1.getPosition()) > self.top_limit_lss1

    def lss0_beyond_bottom_limit(self):
        return int(self.lss0.getPosition()) > self.bottom_limit_lss0

    def lss0_beyond_top_limit(self):
        return int(self.lss0.getPosition()) < self.top_limit_lss0

    def down_step(self, lss):
        lss.wheelRPM(10)

    def up_step(self, lss):
        lss.wheelRPM(-10)

    def stop_wheel(self, lss):
        lss.wheelRPM(0)
    
    def clear_button_press_flags(self):
        self.joystick_button_pressed = self.reset_button_pressed = \
        self.run_predefined_path_button_pressed = self.map_button_pressed = 0 

    def update_simultaneous_button_press_conditions(self):

        self.simultaneous_button_press_conditions = [
            (self.run_predefined_path_button_pressed and self.map_button_pressed),
            (self.reset_button_pressed and self.run_predefined_path_button_pressed),
            (self.joystick_button_pressed and self.run_predefined_path_button_pressed),
            (self.joystick_button_pressed and self.reset_button_pressed),
            (self.joystick_button_pressed and self.reset_button_pressed and self.run_predefined_path_button_pressed),
            (self.reset_button_pressed and self.map_button_pressed),
            (self.reset_button_pressed and self.run_predefined_path_button_pressed and self.map_button_pressed),
            (self.joystick_button_pressed and self.map_button_pressed),
            (self.joystick_button_pressed and self.run_predefined_path_button_pressed and self.map_button_pressed),
            (self.joystick_button_pressed and self.reset_button_pressed and self.map_button_pressed),
            (self.joystick_button_pressed and self.reset_button_pressed and self.run_predefined_path_button_pressed and self.map_button_pressed)
        ]

    def read_values_from_serial(self):
        if (not self.wait_to_read_values_from_serial):
            time.sleep(1)
            self.ser_obj_controller.flushInput()  # Flush incoming serial data
            self.wait_to_read_values_from_serial = True
        else:
            received_vals = self.ser_obj_controller.readline().decode().strip()
            values = received_vals.split(',')
            if (len(values) == 6):
                self.x_analog_value = int(values[0])
                self.z_analog_value = int(values[1])
                self.joystick_button_pressed = int(values[2])
                self.reset_button_pressed = int(values[3])
                self.run_predefined_path_button_pressed = int(values[4])
                self.map_button_pressed = int(values[5])
                # self.get_logger().info(f'read serial vals: {self.x_analog_value, self.z_analog_value, self.joystick_button_pressed, self.reset_button_pressed, self.run_predefined_path_button_pressed, self.map_button_pressed }')

                self.update_simultaneous_button_press_conditions()
            else:
                self.x_analog_value = self.z_analog_value = self.joystick_button_pressed = self.reset_button_pressed = \
                self.run_predefined_path_button_pressed = self.map_button_pressed = None
    
    def move_arm_north(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss1_beyond_top_limit() and not self.lss0_beyond_top_limit()):

                self.get_logger().info('Moving upwards (NORTH)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.down_step(self.lss1)
                self.up_step(self.lss0)

            else:
                self.get_logger().info('Maximum top position in both or either of the servos has been reached: Upwards motion not allowed')
                self.get_logger().info(f'LSS1 reached maximum top position: {self.lss1_beyond_top_limit()}')
                self.get_logger().info(f'LSS0 reached maximum top position: {self.lss0_beyond_top_limit()}')

        elif (self.current_system_state == 'map'):
        
            self.down_step(self.lss1)
            self.up_step(self.lss0)


    
    def move_arm_south(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss1_beyond_bottom_limit() and not self.lss0_beyond_bottom_limit()):

                self.get_logger().info('Moving downwards (SOUTH)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.up_step(self.lss1)
                self.down_step(self.lss0)

            else:
                self.get_logger().info('Maximum bottom position in both or either of the servos has been reached: downwards motion not allowed')
                self.get_logger().info(f'LSS1 reached maximum bottom position: {self.lss1_beyond_bottom_limit()}')
                self.get_logger().info(f'LSS0 reached maximum bottom position: {self.lss0_beyond_bottom_limit()}')

        elif (self.current_system_state == 'map'):
    
            self.up_step(self.lss1)
            self.down_step(self.lss0)

        
    def move_arm_east(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss1_beyond_bottom_limit() and not self.lss0_beyond_top_limit()):

                self.get_logger().info('Moving to the right (EAST)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.up_step(self.lss1)
                self.up_step(self.lss0)

            else:
                self.get_logger().info('Maximum positions in both or either of the servos has been reached: Eastward motion not allowed')
                self.get_logger().info(f'LSS1 reached maximum bottom position: {self.lss1_beyond_bottom_limit()}')
                self.get_logger().info(f'LSS0 reached maximum top position: {self.lss0_beyond_top_limit()}')

        elif (self.current_system_state == 'map'):

            self.up_step(self.lss1)
            self.up_step(self.lss0)

    
    def move_arm_west(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss1_beyond_top_limit() and not self.lss0_beyond_bottom_limit()):

                self.get_logger().info('Moving to the left (WEST)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.down_step(self.lss1)
                self.down_step(self.lss0)

            else:
                self.get_logger().info('Maximum positions in both or either of the servos has been reached: Westward motion not allowed')
                self.get_logger().info(f'LSS1 reached maximum top position: {self.lss1_beyond_top_limit()}')
                self.get_logger().info(f'LSS0 reached maximum bottom position: {self.lss0_beyond_bottom_limit()}')
        
        if (self.current_system_state == 'map'):

            self.down_step(self.lss1)
            self.down_step(self.lss0)

    
    def move_arm_northeast(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss0_beyond_top_limit()):

                self.get_logger().info('Moving in the right-up direction (Northeast)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.up_step(self.lss0)

            else:
                self.get_logger().info(f'LSS0 reached top position: {self.lss0_beyond_top_limit()} (Northeast motion not allowed)')

        elif (self.current_system_state == 'map'):
        
            self.up_step(self.lss0)


    def move_arm_southeast(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss1_beyond_bottom_limit()):

                self.get_logger().info('Moving in the right-down direction (Southeast)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.up_step(self.lss1)

            else:
                self.get_logger().info(f'LSS1 reached bottom position: {self.lss0_beyond_bottom_limit()} (Southeast motion not allowed)')

        elif (self.current_system_state == 'map'):

            self.up_step(self.lss1)


    def move_arm_northwest(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss1_beyond_top_limit()):

                self.get_logger().info('Moving in the left-up direction (Northwest)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.down_step(self.lss1)

            else:
                self.get_logger().info(f'LSS1 reached top position: {self.lss0_beyond_top_limit()} (Northwest motion not allowed)')
        
        elif (self.current_system_state == 'map'):
        
            self.down_step(self.lss1)

    def move_arm_southwest(self):

        if (self.current_system_state == 'joystick_arm_control'):

            if (not self.lss0_beyond_bottom_limit()):

                self.get_logger().info('Moving in the left-down direction (Southwest)')
                self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
                self.down_step(self.lss0)

            else:
                self.get_logger().info(f'LSS0 reached bottom position: {self.lss0_beyond_bottom_limit()} (Southwest motion not allowed)')
        
        elif (self.current_system_state == 'map'):
            self.down_step(self.lss0)

        
    def pause_arm_movement(self):
            
        if (self.current_system_state == 'joystick_arm_control'):
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.get_logger().info('Joystick at middle placement (Motion paused)')
            self.stop_wheel(self.lss1)
            self.stop_wheel(self.lss0)
        else:
            self.stop_wheel(self.lss1)
            self.stop_wheel(self.lss0)

                  
    def control_arm_with_joystick(self):

        if (self.reset_button_pressed and self.current_system_state == 'joystick_arm_control'):
            self.lss1.wheelRPM(0)
            self.lss0.wheelRPM(0)
            return
        
        if (self.map_button_pressed and self.current_system_state == 'map'):
            self.send_map_button_presses()
    
        if (self.x_analog_value is not None and self.z_analog_value is not None):
                
                # Joystick at North (UP) placement
                if ((0 <= self.x_analog_value <= self.zero_value_deadzone) and \
                    (512 - self.diagonal_threshold < self.z_analog_value < 512 + self.diagonal_threshold)):

                    self.move_arm_north()

                # Joystick at South (DOWN) placement 
                elif ((self.x_analog_value == 1023) and \
                    (512 - self.diagonal_threshold < self.z_analog_value < 512 + self.diagonal_threshold)):
                
                    self.move_arm_south()
                    
                # Joystick at East (RIGHT) placement 
                elif ((512 - self.diagonal_threshold < self.x_analog_value < 512 + self.diagonal_threshold) and \
                    (0 <= self.z_analog_value <= self.zero_value_deadzone)):

                    self.move_arm_east()

                # Joystick at West (LEFT) placement
                elif ((512 - self.diagonal_threshold < self.x_analog_value < 512 + self.diagonal_threshold) and \
                    (self.z_analog_value == 1023)):
                    
                    self.move_arm_west()

                # Joystick at Northeast (RIGHT-UP DIAGONAL) placement
                elif ((0 <= self.x_analog_value <= 512 - self.diagonal_threshold) and \
                    (0 <= self.z_analog_value <= 512 - self.diagonal_threshold)):

                    self.move_arm_northeast()
                    
                # Joystick at Southeast (RIGHT-DOWM DIAGONAL) placement
                elif ((512 + self.diagonal_threshold <= self.x_analog_value <= 1023) and \
                    (0 <= self.z_analog_value <= 512 - self.diagonal_threshold)):
                    
                    self.move_arm_southeast()

                # Joystick at Northwest (LEFT-UP DIAGONAL) placement
                elif ((0 <= self.x_analog_value <= 512 - self.diagonal_threshold) and \
                    (512 + self.diagonal_threshold <= self.z_analog_value <= 1023)):

                    self.move_arm_northwest()

                # Joystick at Southwest (LEFT-DOWN DIAGONAL) placement
                elif (( 512 + self.diagonal_threshold <= self.x_analog_value <= 1023) and \
                    (512 + self.diagonal_threshold <= self.z_analog_value <= 1023)):

                    self.move_arm_southwest()

                # Joystick at middle placement
                else:
                    self.pause_arm_movement()



    def check_state_callback(self, msg):
        self.previous_system_state = self.current_system_state
        self.current_system_state = msg.data
        self.get_logger().info(f'Set the current system state variable to: {self.current_system_state} and the previous system state to: {self.previous_system_state}')
        if (self.current_system_state == 'standby'):
            self.standby_logger_printed = False
            return
        
    def cleanup_serial(self):
        if self.ser_obj_controller.is_open:
            self.ser_obj_controller.close()
            self.get_logger().info('Serial connection closed')
    
    def send_map_button_presses(self):
        msg = String()
        msg.data = "1"
        self.get_logger().info('Publishing a message to the map_button_pressed topic')
        self.send_map_button_press_publisher.publish(msg)
    
    def send_system_state_request(self, request):
        msg = String()
        msg.data = request
        self.get_logger().info(f'The following request was sent to the state_manager node: {request}, on the topic: system_state_request')
        self.set_system_state_by_request_publisher.publish(msg)

    def calculate_time_to_run_rail_system(self, rpm):
        revolutions = self.distance_to_travel/self.circumference_of_wheel
        time = (60/np.abs(rpm))*revolutions
        return time

    def move_rail_system(self, rpm, target_pos):

        self.get_logger().info(f'Moving DuoArm to position {target_pos} on the y - axis')
        self.lss2.wheelRPM(rpm)
        time.sleep(self.calculate_time_to_run_rail_system(rpm))
        self.lss2.wheelRPM(0)
        self.rail_position = target_pos

    def activate_rail_system(self):

        if (self.rail_position == "A"):
            self.move_rail_system(30, "B")
            self.save_last_rail_position()
            self.get_logger().info("Target position reached -> Exiting the joystick_rail_control state")
            self.send_system_state_request('joystick_arm_control')
            self.wait_for_state_change('joystick_arm_control')
            self.ser_obj_controller.reset_input_buffer()

        elif (self.rail_position == "B"):
            self.move_rail_system(-30, "A")
            self.save_last_rail_position()
            self.get_logger().info("Target position reached -> Exiting the joystick_rail_control state")
            self.send_system_state_request('joystick_arm_control')
            self.wait_for_state_change('joystick_arm_control')
            self.ser_obj_controller.reset_input_buffer()

        else: 
            self.get_logger().info('Check the boundary_path_and_rail_position.json file for an invalid rail position value (The value should either be A or B)')
            return
    
    def wait_for_state_change(self, expected_state):
        while True:
            if (self.current_system_state == expected_state):
                break

  
def main():
    rclpy.init()

    hic_obj = HardwareInterfaceController()

    try:

        hic_obj_node_spin_thread = threading.Thread(target=rclpy.spin, args=(hic_obj,))
        hic_obj_node_spin_thread.start()
        
        while True:
            
            hic_obj.read_values_from_serial()

            if (hic_obj.previous_system_state == 'map' and hic_obj.current_system_state == 'standby' and not hic_obj.boundaries_and_last_rail_position_data_updated):
                hic_obj.set_boundaries_and_last_rail_position_data()

            if (hic_obj.previous_system_state == 'standby' and hic_obj.current_system_state == 'map'):
                hic_obj.boundaries_and_last_rail_position_data_updated = False

            if (any(hic_obj.simultaneous_button_press_conditions)):
                hic_obj.get_logger().info('Multiple buttons were pressed simultaneously -> Button presses was ignored')
                hic_obj.clear_button_press_flags()

            if(hic_obj.joystick_button_pressed and hic_obj.boundaries_and_last_rail_position_data is not None):
                hic_obj.send_system_state_request('joystick_control')

            elif (hic_obj.joystick_button_pressed and hic_obj.boundaries_and_last_rail_position_data is None):
                hic_obj.get_logger().info('A mapping sequence have to be executed to control the arm with the joystick and/or activate the rail system')

            if (hic_obj.current_system_state == 'joystick_arm_control'or hic_obj.current_system_state == 'map'):
                hic_obj.control_arm_with_joystick()
            
            if(hic_obj.current_system_state == 'joystick_rail_control'):
                hic_obj.get_logger().info('Activated the rail system')
                hic_obj.activate_rail_system()
            
            if(hic_obj.reset_button_pressed):
                hic_obj.send_system_state_request('standby')
            
            if (hic_obj.current_system_state == 'standby' and hic_obj.standby_logger_printed == False):
                hic_obj.get_logger().info('Currently in the standby state')
                hic_obj.standby_logger_printed = True

            if(hic_obj.run_predefined_path_button_pressed and hic_obj.boundaries_and_last_rail_position_data is not None):
               hic_obj.send_system_state_request('run_predefined_path')
            
            elif (hic_obj.run_predefined_path_button_pressed and hic_obj.boundaries_and_last_rail_position_data is None):
                hic_obj.get_logger().info('A mapping sequence have to be executed to run a predefined path')
            
            if(hic_obj.map_button_pressed and not hic_obj.current_system_state == 'map'):
                hic_obj.send_system_state_request('map')

            
    except KeyboardInterrupt:

        GPIO.cleanup()
        hic_obj.cleanup_serial()
    
    finally:

        hic_obj_node_spin_thread.join()
        hic_obj.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




        
















