#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import tkinter as tk
from tkinter import simpledialog

class CommandPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gui_command_publisher', anonymous=True)
        self.publisher = rospy.Publisher('command_topic', String, queue_size=10)
        
        # Setup GUI
        self.root = tk.Tk()
        self.root.geometry("300x100")
        self.setup_gui()
        self.root.mainloop()
    
    def setup_gui(self):
        self.text_entry = tk.Entry(self.root)
        self.text_entry.pack(pady=20)
        
        self.send_button = tk.Button(self.root, text="Send Command", command=self.send_command)
        self.send_button.pack()
        
    def send_command(self):
        # Read from text entry and publish
        command = self.text_entry.get()
        self.publisher.publish(command)
        print(f"Sent command: {command}")
        
if __name__ == "__main__":
    try:
        cp = CommandPublisher()
    except rospy.ROSInterruptException:
        pass
