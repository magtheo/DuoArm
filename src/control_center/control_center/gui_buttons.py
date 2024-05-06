import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROS2GUI(Node):
    def __init__(self, master):
        super().__init__('gui_node')
        self.master = master
        self.master.title("ROS2 Control Panel")

        self.button_press_publisher = self.create_publisher(String, 'map_button_press', 10)
        self.state_publisher = self.create_publisher(String, 'action_controller_state', 10)

        self.map_button = tk.Button(master, text="Map Button Press", command=self.simulate_map_button_press)
        self.map_button.pack(pady=10)

        self.set_map_state = tk.Button(master, text="Set State to 'Map'", command=self.set_state_to_map)
        self.set_map_state.pack(pady=10)

        # Adding a new button for running the path
        self.run_path_button = tk.Button(master, text="Run Path", command=self.run_path)
        self.run_path_button.pack(pady=10)

    def simulate_map_button_press(self):
        msg = String()
        msg.data = 'button_pressed'
        self.button_press_publisher.publish(msg)
        print("Simulated Map Button Press")

    def set_state_to_map(self):
        msg = String()
        msg.data = 'map'
        self.state_publisher.publish(msg)
        print("State set to 'Map'")

    def run_path(self):
        msg = String()
        msg.data = 'path'
        self.state_publisher.publish(msg)
        print("Path execution started")

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    root = tk.Tk()
    gui_node = ROS2GUI(root)
    
    # Start ROS2 node in a separate thread
    thread = threading.Thread(target=ros_spin, args=(gui_node,))
    thread.start()

    # Start the tkinter GUI main loop
    root.mainloop()

    # Once the GUI is closed, shutdown ROS and wait for thread to finish
    gui_node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
<<<<<<< HEAD:src/control_center/control_center/gui_buttons.py
    main()
=======
    main()
>>>>>>> f99a8527a704b15c0031485ea0da93b97bf6bd83:src/controll_center/controll_center/gui_buttons.py
