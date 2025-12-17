#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tkinter as tk

class SixButtonControlNode(Node):
    def __init__(self):
        super().__init__('six_button_control_node')

        # ROS2 publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_state_pub = self.create_publisher(String, '/robot_state', 10)

        # Default speeds
        self.linear_x_speed = 0.3
        self.linear_y_speed = 0.3
        self.angular_speed = 1.0

        # Launch GUI
        self.init_window()
        self.get_logger().info('Six-button control node started')

    def init_window(self):
        self.root = tk.Tk()
        self.root.title("Six-Button Control")
        self.root.geometry("500x350")

        tk.Label(self.root, text="Click buttons to control x, y, rotation").grid(row=0, column=0, columnspan=3, pady=5)

        # Linear x speed scale
        tk.Label(self.root, text="Linear X speed").grid(row=1, column=0)
        self.linear_x_scale = tk.Scale(self.root, from_=0.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL, length=200)
        self.linear_x_scale.set(self.linear_x_speed)
        self.linear_x_scale.grid(row=1, column=1, columnspan=2)

        # Linear y speed scale
        tk.Label(self.root, text="Linear Y speed").grid(row=2, column=0)
        self.linear_y_scale = tk.Scale(self.root, from_=0.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL, length=200)
        self.linear_y_scale.set(self.linear_y_speed)
        self.linear_y_scale.grid(row=2, column=1, columnspan=2)

        # Angular speed scale
        tk.Label(self.root, text="Angular speed").grid(row=3, column=0)
        self.angular_scale = tk.Scale(self.root, from_=0.0, to=3.0, resolution=0.1, orient=tk.HORIZONTAL, length=200)
        self.angular_scale.set(self.angular_speed)
        self.angular_scale.grid(row=3, column=1, columnspan=2)

        # Buttons
        btn_cfg = {"width":12, "height":2}

        tk.Button(self.root, text="Forward (x+)", bg="lightblue", **btn_cfg, command=self.forward).grid(row=4, column=1, pady=5)
        tk.Button(self.root, text="Backward (x-)", bg="lightblue", **btn_cfg, command=self.backward).grid(row=6, column=1, pady=5)
        tk.Button(self.root, text="Left (y+)", bg="lightgreen", **btn_cfg, command=self.left).grid(row=5, column=0, pady=5)
        tk.Button(self.root, text="Right (y-)", bg="lightgreen", **btn_cfg, command=self.right).grid(row=5, column=2, pady=5)
        tk.Button(self.root, text="Turn Left", bg="lightcoral", **btn_cfg, command=self.turn_left).grid(row=4, column=0, pady=5)
        tk.Button(self.root, text="Turn Right", bg="lightcoral", **btn_cfg, command=self.turn_right).grid(row=4, column=2, pady=5)

        tk.Button(self.root, text="Stop", bg="yellow", **btn_cfg, command=self.stop_robot).grid(row=5, column=0, columnspan=5, pady=10)
        tk.Button(self.root, text="Exit", bg="gray", **btn_cfg, command=self.shutdown).grid(row=7, column=1, pady=5)

        # Status label
        self.status_label = tk.Label(self.root, text="Linear x:0.00, y:0.00, Angular:0.00")
        self.status_label.grid(row=8, column=0, columnspan=3, pady=10)

    # ========================
    # Button callbacks
    # ========================
    def forward(self):
        self.publish_cmd(self.linear_x_scale.get(), 0.0, 0.0)

    def backward(self):
        self.publish_cmd(-self.linear_x_scale.get(), 0.0, 0.0)

    def left(self):
        self.publish_cmd(0.0, self.linear_y_scale.get(), 0.0)

    def right(self):
        self.publish_cmd(0.0, -self.linear_y_scale.get(), 0.0)

    def turn_left(self):
        self.publish_cmd(0.0, 0.0, self.angular_scale.get())

    def turn_right(self):
        self.publish_cmd(0.0, 0.0, -self.angular_scale.get())

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0, 0.0)
        self.get_logger().info("Emergency stop")

    # ========================
    # Publish helper
    # ========================
    def publish_cmd(self, x, y, angular):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

        state_msg = String()
        if x==0.0 and y==0.0 and angular==0.0:
            state_msg.data = "IDLE"
        else:
            state_msg.data = f"MOVE x={x:.2f}, y={y:.2f}, angular={angular:.2f}"
        self.robot_state_pub.publish(state_msg)

        # Update status
        self.status_label.config(text=f"Linear x:{x:.2f}, y:{y:.2f}, Angular:{angular:.2f}")

    # ========================
    # Shutdown
    # ========================
    def shutdown(self):
        self.get_logger().info("Shutting down node")
        self.root.destroy()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SixButtonControlNode()
    node.root.mainloop()

if __name__ == "__main__":
    main()
