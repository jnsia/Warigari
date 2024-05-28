import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from my_py_interface.msg import Lidar  # 커스텀 메시지 임포트
import time
import math

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_distance_listener')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.publisher = self.create_publisher(Lidar, '/object_detection', 10)
    
    def callback(self, data):
        detection = Lidar()
        detection.front = self.detect_objects_fb(data, 170, 180)  # 전
        detection.left = self.detect_objects_lr(data, -160, -20)  # 좌
        detection.right = self.detect_objects_lr(data, 20, 160)   # 우
        detection.back = self.detect_objects_fb(data, -10, 10)    # 후
        self.publisher.publish(detection)

    def detect_objects_lr(self, data, start_angle, end_angle):
        start_index = int((math.radians(start_angle) - data.angle_min) / data.angle_increment)
        end_index = int((math.radians(end_angle) - data.angle_min) / data.angle_increment)
        for i in range(start_index, end_index + 1):
            if i < 0 or i >= len(data.ranges):  # 범위 보호
                continue
            if data.ranges[i] < 0.7:  # 임계값 내에 물치 감지시 True값 반환
                return True
        return False
    
    def detect_objects_fb(self, data, start_angle, end_angle):
        start_index = int((math.radians(start_angle) - data.angle_min) / data.angle_increment)
        end_index = int((math.radians(end_angle) - data.angle_min) / data.angle_increment)
        for i in range(start_index, end_index + 1):
            if i < 0 or i >= len(data.ranges):  # 범위 보호
                continue
            if data.ranges[i] < 1:  # 임계값 내에 물치 감지시 True값 반환
                return True
        return False

    
def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    rclpy.spin(lidar_listener)
    lidar_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
