#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import tkinter as tk
from tkinter import simpledialog

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('gui_command_publisher')
        self.publisher = self.create_publisher(String, 'command_topic', 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % command)
        

def ros_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    command_publisher = CommandPublisher()

    # Start ROS node in a separate thread
    thread = threading.Thread(target=ros_thread, args=(command_publisher,))
    thread.daemon = True
    thread.start()

    # Setup GUI in the main thread
    root = tk.Tk()
    root.title("Command Publisher")
    root.geometry("300x100")

    text_entry = tk.Entry(root)
    text_entry.pack(pady=20)

    def send_command():
        command = text_entry.get()
        command_publisher.send_command(command)

    send_button = tk.Button(root, text="Send Command", command=send_command)
    send_button.pack()

    root.mainloop()

if __name__ == '__main__':
    main()
