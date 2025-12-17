import rclpy
import struct
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import numpy as np

from get_dispose_serial.myserial import AsyncSerial_t 
from get_dispose_serial.in_dispose import dispose_serial_init
class SerialDisposeNode(Node):
    def __init__(self):
        super().__init__('serial_dispose_node')
        self.serial = dispose_serial_init(
            port="/dev/tnt1",
            baudrate=115200,
            callback=self.on_odom
        )
        self.publisher_ = self.create_publisher(Vector3Stamped, 'lidar_position', 10)
        self.publisher_odom = self.create_publisher(Vector3Stamped, 'odom_data', 10)
        #self.sub_cmd_vel = self.create_subscription(Twist, self.get_parameter('cmd_vel'), 10)
        self.topic_map = {
            "/cmd_vel": {
                "msg_type": Twist,
                "func": 0xA0,
                "encoder": self.encode_cmd_vel
            },
            "/robot_state": {
                "msg_type": String,
                "func": 0xA0,
                "encoder": self.encode_robot_state
            }
        }
        self.subs = []
        for topic, cfg in self.topic_map.items():
            sub = self.create_subscription(
                cfg["msg_type"],
                topic,
                lambda msg, t=topic: self.topic_callback(t, msg),
                10
            )
            self.subs.append(sub)
    def build_frame(self, func: int, payload: bytes) -> bytes:
        return b'\xFA' + bytes([func]) + payload    
    def topic_callback(self, topic, msg):
        cfg = self.topic_map[topic]
        func = cfg["func"]
        payload = cfg["encoder"](msg)
        frame = self.build_frame(func, payload)
        self.serial.write(frame)
   
    def encode_cmd_vel(self, msg: Twist) -> bytes:
        return struct.pack("<fff",msg.linear.x,msg.linear.y,msg.angular.z)
    def encode_robot_state(self, msg: String) -> bytes:
        return struct.pack("<fff",0.0,0.0,0.0)
    
    def on_odom(self, x: float, y: float, yaw: float):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = yaw
        self.publisher_odom.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    serial_dispose_node = SerialDisposeNode()
    rclpy.spin(serial_dispose_node)
    serial_dispose_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()