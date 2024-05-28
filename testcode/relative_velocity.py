#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # Boolean 메시지 타입 import
import time
import math


class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_distance_listener')

    def callback(self,):
        pass

    def publish_detection(self, detection):
        msg = Bool()
        msg.data = detection
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    rclpy.spin(lidar_listener)
    lidar_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
