import serial
from .lss import *
import RPi.GPIO as GPIO
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String


class JoystickController(Node):
    def __init__(self):
        super().__init__('jc_obj')

        self.temp_system_state = 'standby'

        self.state_subscriber = self.create_subscription(String, 'action_controller_state', self.check_state_callback, 10)
        self.set_joystick_control_state_publisher = self.create_publisher(String, 'joystick_control_state_request', 10)
        self.set_system_state_rst_or_rpp_publisher = self.create_publisher(String, 'reset_or_rpp_state_request', 10)
        self.set_joystick_arm_control_state_publisher = self.create_publisher(String, 'joystick_arm_control_state_request', 10)
        self.set_system_state_map_publisher = self.create_publisher(String, 'map_state_request', 10)
        self.send_map_button_press_publisher = self.create_publisher(String, 'map_button_pressed', 10)
        self.set_system_state_by_request_publisher = self.create_publisher(String, 'system_state_request', 10)

        """Attribute related to joystick button pressing:"""
        self.joystick_button_pressed = 0

        """Attribute related to reset/"Stop the active process" button pressing:"""
        self.reset_button_pressed = 0

        """Attribute related to reset/"Stop the active process" button pressing:"""
        self.run_predefined_path_button_pressed = 0

        """Attribute related to "mapping" button pressing:"""
        self.map_button_pressed = 0

        """Attributes related to communication between the Arduino MEGA, Raspberry PI, LSS adapter board and LSS motors:"""
        self.CST_LSS_Port = "/dev/ttyUSB0"		
        self.CST_LSS_Baud = LSS_DefaultBaud
        initBus(self.CST_LSS_Port, self.CST_LSS_Baud)
        self.ser_obj = serial.Serial('/dev/ttyACM0', 115200)

        """Attributes related to the received analog values:"""
        self.x_analog_value = 0
        self.z_analog_value = 0
        self.mid_threshold = 30
        self.diagonal_threshold = 255

        """Attributes related to the LSS motors:"""
        self.top_limit_lss1 = -60
        self.bottom_limit_lss1 = 1680
        self.top_limit_lss0 = 640
        self.bottom_limit_lss0 = -1740
        self.lss0 = LSS(0)
        self.lss1 = LSS(1)
        self.lss2 = LSS(2)
        
        """Attributes related to the rail system:"""
        self.position_threshold = 5
        self.position_A_range = range(3586 - self.position_threshold, 3586 + self.position_threshold) 
        self.position_B_range = range(0 - self.position_threshold, 0 + self.position_threshold)


        """Attributes related to button pressing cases:"""
        self.simultaneous_button_press_conditions = [
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

    
    def read_values_from_serial(self):

        received_vals = self.ser_obj.readline().decode().strip()
        values = received_vals.split(',')
        if (len(values) == 6):
            self.x_analog_value = int(values[0])
            self.z_analog_value = int(values[1])
            self.joystick_button_pressed = int(values[2])
            self.reset_button_pressed = int(values[3])
            self.run_predefined_path_button_pressed = int(values[4])
            self.map_button_pressed = int(values[5])
        else:
            self.get_logger().info(f'Invalid number of values received: {values}')
            self.x_analog_value = self.z_analog_value = self.joystick_button_pressed = self.reset_button_pressed = \
            self.run_predefined_path_button_pressed = self.map_button_pressed = None
    
    def move_arm_north(self):

        if (not self.lss1_beyond_top_limit() and not self.lss0_beyond_top_limit()):

            self.get_logger().info('Moving upwards (NORTH)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.down_step(self.lss1)
            self.up_step(self.lss0)

        else:
            self.get_logger().info('Maximum top position in both or either of the servos has been reached: Upwards motion not allowed')
            self.get_logger().info(f'LSS1 reached maximum top position: {self.lss1_beyond_top_limit()}')
            self.get_logger().info(f'LSS0 reached maximum top position: {self.lss0_beyond_top_limit()}')

    
    def move_arm_south(self):

        if (not self.lss1_beyond_bottom_limit() and not self.lss0_beyond_bottom_limit()):

            self.get_logger().info('Moving downwards (SOUTH)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.up_step(self.lss1)
            self.down_step(self.lss0)

        else:
            self.get_logger().info('Maximum bottom position in both or either of the servos has been reached: downwards motion not allowed')
            self.get_logger().info(f'LSS1 reached maximum bottom position: {self.lss1_beyond_bottom_limit()}')
            self.get_logger().info(f'LSS0 reached maximum bottom position: {self.lss0_beyond_bottom_limit()}')
        
    def move_arm_east(self):

        if (not self.lss1_beyond_bottom_limit() and not self.lss0_beyond_top_limit()):

            self.get_logger().info('Moving to the right (EAST)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.up_step(self.lss1)
            self.up_step(self.lss0)

        else:
            self.get_logger().info('Maximum positions in both or either of the servos has been reached: Eastward motion not allowed')
            self.get_logger().info(f'LSS1 reached maximum bottom position: {self.lss1_beyond_bottom_limit()}')
            self.get_logger().info(f'LSS0 reached maximum bottom position: {self.lss0_beyond_top_limit()}')
    
    def move_arm_west(self):

        if (not self.lss1_beyond_top_limit() and not self.lss0_beyond_bottom_limit()):

            self.get_logger().info('Moving to the left (WEST)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.up_step(self.lss1)
            self.up_step(self.lss0)

        else:
            self.get_logger().info('Maximum positions in both or either of the servos has been reached: Westward motion not allowed')
            self.get_logger().info(f'LSS1 reached maximum bottom position: {self.lss1_beyond_top_limit()}')
            self.get_logger().info(f'LSS0 reached maximum bottom position: {self.lss0_beyond_bottom_limit()}')

    
    def move_arm_northeast(self):

        if (not self.lss0_beyond_top_limit()):

            self.get_logger().info('Moving in the right-up direction (Northeast)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.up_step(self.lss0)

        else:
            self.get_logger().info(f'LSS0 reached top position: {self.lss0_beyond_top_limit()} (Northeast motion not allowed)')

    def move_arm_southeast(self):

        if (not self.lss1_beyond_bottom_limit()):

            self.get_logger().info('Moving in the right-down direction (Southeast)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.up_step(self.lss1)

        else:
            self.get_logger().info(f'LSS1 reached bottom position: {self.lss0_beyond_bottom_limit()} (Southeast motion not allowed)')

    def move_arm_northwest(self):

        if (not self.lss1_beyond_top_limit()):

            self.get_logger().info('Moving in the left-up direction (Northwest)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.up_step(self.lss1)

        else:
            self.get_logger().info(f'LSS1 reached top position: {self.lss0_beyond_top_limit()} (Northwest motion not allowed)')

    def move_arm_southwest(self):

        if (not self.lss0_beyond_bottom_limit()):

            self.get_logger().info('Moving in the left-down direction (Southwest)')
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.down_step(self.lss0)

        else:
            self.get_logger().info(f'LSS0 reached bottom position: {self.lss0_beyond_bottom_limit()} (Southwest motion not allowed)')
        
    def pause_arm_movement(self):
            
            self.get_logger().info(f'(x_analog_value, z_analog_value) -> ({self.x_analog_value, self.z_analog_value})')
            self.get_logger().info('Joystick at middle placement (Motion paused)')
            self.stop_wheel(self.lss1)
            self.stop_wheel(self.lss0)
                  
    def control_arm_with_joystick(self):

        if (self.x_analog_value is not None and self.z_analog_value is not None):
                
                # Joystick at North (UP) placement
                if ((self.x_analog_value == 0) and \
                    (512 - self.diagonal_threshold < self.z_analog_value < 512 + self.diagonal_threshold)):

                    self.move_arm_north()

                # Joystick at South (DOWN) placement 
                elif ((self.x_analog_value == 1023) and \
                    (512 - self.diagonal_threshold < self.z_analog_value < 512 + self.diagonal_threshold)):
                
                    self.move_arm_south()
                    
                # Joystick at East (RIGHT) placement 
                elif ((512 - self.diagonal_threshold < self.x_analog_value < 512 + self.diagonal_threshold) and \
                    (self.z_analog_value == 0)):

                    self.move_arm_east()

                # Joystick at West (LEFT) placement
                elif ((512 - self.diagonal_threshold < self.x_analog_value < self.diagonal_threshold) and \
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
                    (512 + self.diagonal_threshold <= self.x_analog_value <= 1023)):

                    self.move_arm_northwest()

                # Joystick at middle placement
                else:
                    self.pause_arm_movement()


    def check_state_callback(self, msg):
        self.temp_system_state = msg.data
        self.get_logger().info(f'Set the temp system state variable to: {self.temp_system_state}')

    # def send_general_joystick_control_state_request(self):
    #     msg = String()
    #     msg.data = 'set joystick control state'
    #     self.get_logger().info('Publishing a request to set the system state to one of the two joystick states')
    #     self.set_joystick_control_state_publisher.publish(msg)
    
    # def send_reset_or_rpp_system_state_request(self, request):
    #     msg = String()
    #     msg.data = request
    #     self.get_logger().info(f'Publishing a request: ({request}) to the action_controller node')
    #     self.reset_system_state_to_standby_publisher.publish(msg)
    
    # def send_joystick_arm_control_state_request(self):
    #     msg = String()
    #     msg.data = 'set joystick_arm_control state'
    #     self.get_logger().info('Publishing a request to set the system state to the joystick_arm_control state')
    #     self.set_joystick_arm_control_state_publisher.publish(msg)
    
    # def send_map_state_request(self):
    #     msg = String()
    #     msg.data = 'set system state to map'
    #     self.get_logger().info('Publishing a request to set the system state to the map state')
    #     self.set_system_state_map_publisher.publish(msg)
    def cleanup_serial(self):
        if self.ser_obj.is_open:
            self.ser_obj.close()
            self.get_logger().info('Serial connection closed')
    
    def send_map_button_presses(self, map_button_pressed):
        msg = String()
        msg.data = map_button_pressed
        self.get_logger().info('Publishing a message to the map_button_pressed topic')
        self.send_map_button_press_publisher.publish(msg)
    
    def send_system_state_request(self, request):
        msg = String()
        msg.data = request
        self.get_logger().info(f'The following request was sent to the action_controller node: {request}, on the topic: system_state_request')
        self.set_system_state_by_request_publisher.publish(msg)

    def activate_rail_system(self):

        match self.lss2.getPosition():

            case self.position_A_range:
                self.get_logger().info('Moving DuoArm to position B on the y - axis')
                self.lss2.wheelRPM(-10)
                time.sleep(6)
                   
            case self.position_B_range:
                self.get_logger().info('Moving DuoArm to position A on the y - axis')
                self.lss2.wheelRPM(10)
                time.sleep(6)

        self.get_logger().info("Target position reached -> Exiting the joystick_rail_control state")
        self.send_system_state_request('joystick_arm_control')

            
    
def main():
    rclpy.init()

    jc_obj = JoystickController()
    try:

        jc_obj_thread = threading.Thread(target=rclpy.spin, args=(jc_obj,))
        jc_obj_thread.start()
 
        while True:

            jc_obj.read_values_from_serial()

            jc_obj.get_logger().info(f'x val: {jc_obj.x_analog_value}\n\
                                     z val: {jc_obj.z_analog_value}\n\
                                        joy button pressed: {jc_obj.joystick_button_pressed}\n\
                                        reset button pressed: {jc_obj.reset_button_pressed}\n\
                                        rpp button pressed: {jc_obj.run_predefined_path_button_pressed}\n\
                                        map button pressed: {jc_obj.map_button_pressed}')

            # if (jc_obj.joystick_button_pressed):

            #     jc_obj.get_logger().info(f'Joystick button pressed: {jc_obj.joystick_button_pressed}')
            #     jc_obj.send_system_state_request('joystick_control')
            
            # if (jc_obj.temp_system_state == 'joystick_arm_control'):

            #     jc_obj.get_logger().info('Currently controlling the arm with a joystick')
            #     #jc_obj.control_arm_with_joystick()
            
            # if (jc_obj.temp_system_state == 'joystick_rail_control'):

            #     jc_obj.get_logger().info('Activated the rail system')
            #     jc_obj.activate_rail_system()
            
            # if (jc_obj.reset_button_pressed):
            #     jc_obj.send_system_state_request('standby')

            # if (jc_obj.run_predefined_path_button_pressed):
            #     jc_obj.send_system_state_request('run_predefined_path')
            
            # if (jc_obj.map_button_pressed):

            #     if (jc_obj.temp_system_state == 'map'):
            #         jc_obj.send_map_button_presses(String(jc_obj.map_button_pressed))

            #     else:
            #         jc_obj.send_system_state_request('map')

            # if (any(jc_obj.simultaneous_button_press_conditions)):
            #     jc_obj.get_logger().info('Multiple buttons were pressed simultaneously -> Button presses was ignored')

    except KeyboardInterrupt:

        GPIO.cleanup()
        jc_obj.cleanup_serial()
    
    finally:

        jc_obj_thread.join()
        jc_obj.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




        
















