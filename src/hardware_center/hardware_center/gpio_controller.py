import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class GPIOController(Node):
    """
    @class GPIOController
    @brief Used to send state to arduino that then powers LED lights with correct state colour.
    """
    def __init__(self):
        """Initialize the GPIOController node. Set up a subscription to listen for state updates from the action controller."""
        super().__init__('gpio_controller')
        self.subscription = self.create_subscription(
            String,
            'system_state',
            self.handle_state_update,
            10)
        
        self.setup_gpio()

        self.previous_state = None  # Initialize previous state as None
        self.state = 'standby'

    def setup_gpio(self):
        """
        Configure the GPIO settings for the Raspberry Pi. Set up three LED pins (red, green, blue) for output
        and initialize them to a standby state.
        """
        GPIO.setmode(GPIO.BCM)
        self.red_led_pin = 17
        self.blue_led_pin = 27
        self.green_led_pin = 22
        GPIO.setup(self.red_led_pin, GPIO.OUT)
        GPIO.setup(self.green_led_pin, GPIO.OUT)
        GPIO.setup(self.blue_led_pin, GPIO.OUT)
        self.set_led_state(red=True, green=False, blue=False) # set to standby 

    def handle_state_update(self, msg):
        """
        Change LED states based on the received state from stateManager.
        """
        self.get_logger().info(f'new state in gpio_controller')
        if self.state != msg.data:
            self.state = msg.data
            if self.state == 'standby':
                self.set_led_state(red=True, green=False, blue=False)
                return
            elif self.state == 'joystick_arm_control':
                self.set_led_state(red=True, green=False, blue=True)  #pink for 'joystick_arm_control'
                return
            elif self.state == 'joystick_rail_control':
                self.set_led_state(red=False, green=True, blue=True)  # Assuming for 'joystick_rail_control'
                return
            elif self.state == 'map':
                self.set_led_state(red=True, green=True, blue=False)  # yellow map
                return
            elif self.state == 'run_predefined_path':
                self.set_led_state(red=False, green=True, blue=False) # green path
                return

    def set_led_state(self, red, green, blue):
        """
        output signal throug the GPIO pins based on the provided boolean values for red, green, and blue.
        The arduino reads these signals and shows the correct colour.
        """
        self.get_logger().info(f'before gpio output')

        GPIO.output(self.red_led_pin, GPIO.HIGH if red else GPIO.LOW)
        GPIO.output(self.green_led_pin, GPIO.HIGH if green else GPIO.LOW)
        GPIO.output(self.blue_led_pin, GPIO.HIGH if blue else GPIO.LOW)

        self.get_logger().info(f'after gpio output')
        return

    def on_shutdown(self):
        """
        Clean up the GPIO settings to ensure the GPIO pins are left in a safe state when the program terminates.
        """
        GPIO.cleanup()

def main(args=None):
    """
    Main function to initialize the ROS node and handle its lifecycle.
    """
    rclpy.init(args=args)
    gpio_controller = GPIOController()
    rclpy.spin(gpio_controller)
    gpio_controller.on_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
