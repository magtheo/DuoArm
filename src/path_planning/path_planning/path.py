
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandListener(Node):
    def __init__(self):
        super().__init__('command_listener')
        self.subscription = self.create_subscription(
            String,
            'command_topic',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info('I heard: "%s"' % command)
        # Implement command handling logic here

def main(args=None):
    rclpy.init(args=args)
    node = CommandListener()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
