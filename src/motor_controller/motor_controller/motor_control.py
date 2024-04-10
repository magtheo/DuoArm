import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import numpy as np


from .lss import *

CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

# Create and open a serial port
initBus(CST_LSS_Port, CST_LSS_Baud)

lss0 = LSS(0)
lss1 = LSS(1)

lss0_act_angle = 0
lss1_act_angle = 0

lss0_act_position = 0
lss1_act_position = 0

received_joint_angles = np.zeros(2)
active_joint_angles = np.zeros(2)
transmission_msg = Float64MultiArray()

class motorControl(Node):
    def __init__(self):
        super().__init__("motor_control")

        self.calc_joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'calculated_joint_angles',
            self.calc_joint_angles_callback,
            10
        )
        self.active_joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 
            'actual_joint_angles',
            10
        )
        
    def calc_joint_angles_callback(self, msg):
        received_joint_angles = msg
        self.get_logger().info(f'Received calculated joint angles {received_joint_angles}')
    
    #def send_actual_joint_angles(self):
    def calc_position(self, angle):
        return angle*10
    
    def calc_angle(self, position):
        return position/10

    def move_servos(self):
        lss0.move(self.calc_position(received_joint_angles[0]))
        lss1.move(self.calc_position(received_joint_angles[1]))
        
        if (lss0.getPosition()/10 == received_joint_angles[0] & lss1.getPosition()/10 == received_joint_angles[1]):
            lss0_act_position = lss0.getPosition()
            lss1_act_position = lss1.getPosition()


        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)

        active_joint_angles[0] = lss0_act_angle
        active_joint_angles[1] = lss1_act_angle

        transmission_msg.data = active_joint_angles
        self.active_joint_angles_publisher.publish(transmission_msg)
        self.get_logger().info('Published active joint angles on the actual_joint_angles topic')

        return transmission_msg.data


def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()


