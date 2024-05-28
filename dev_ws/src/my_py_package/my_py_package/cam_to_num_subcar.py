import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import cv2
import numpy as np
from picamera2 import Picamera2

def get_candidate(image, width, height):
    # BGR 이미지를 HSV로 변환
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 블러링으로 이미지를 부드럽게 변환
    blurred_image = cv2.medianBlur(hsv_image,3)
    kernel = np.ones((5, 5), np.uint8)
    blurred_image = cv2.erode(blurred_image, kernel, iterations=1)
    # 검은색 범위를 정의 (여기에서는 Hue 0-180, Saturation 0-50, Value 0-75)
    black_mask = cv2.inRange(blurred_image, (0, 0, 0), (180, 255, 38))
    res = [0]
    i = 0
    while i < width:
        j = 0
        while i + j < width and (black_mask[height-1][i+j] == 255 or black_mask[height-20][i+j] == 255):  # 이진 마스크에서 흰색(255) 픽셀을 검은색으로 간주
            j += 1
        if j > 4:  # 연속된 검은색 픽셀이 5개 이상일 때만 중앙점을 계산
            res.append(i + j // 2)
        i += j + 1
    res += [width]
    return res

def get_error(pl1, pl2):
    return abs(pl1[0]-pl2[0]) + abs(pl1[1]-pl2[1])

def get_now_x(image, width, height, prev_x):
    candidate = get_candidate(image, width, height)
    now_x = candidate[:2]
    min_sum_error = get_error(now_x, prev_x)
    for i in range(len(candidate)-1):
        tmp_x = candidate[i:i+2]
        tmp_sum_error = get_error(tmp_x, prev_x)
        if tmp_sum_error < min_sum_error:
            min_sum_error = tmp_sum_error
            now_x = tmp_x
    return now_x

class CamToNumPublisher(Node):
    def __init__(self):
        super().__init__('cam_to_num')
        self.picam2 = Picamera2()
        self.width, self.height = (640, 120)
        self.camConfig = self.picam2.create_preview_configuration(main={"size": (self.width, self.height)},controls={"FrameDurationLimits": (16666, 16666)})
        self.picam2.configure(self.camConfig)
        self.picam2.start()

        self.publisher = self.create_publisher(Int16MultiArray, '/cam_to_num',10)
        timer_period = 0.016
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.prev_x = [0, self.width]
        self.now_x = [0, self.width]
        self.msg = Int16MultiArray()

    def timer_callback(self):
        image = self.picam2.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        self.prev_x = self.now_x
        self.now_x = get_now_x(image, self.width, self.height, self.prev_x)
        
        self.msg.data = self.now_x
        self.publisher.publish(self.msg)
        cv2.line(image,(self.now_x[0],self.height-1), (self.now_x[0],self.height-10), (0, 255, 0), 3)        
        cv2.line(image,(self.now_x[1],self.height-1), (self.now_x[1],self.height-10), (0, 255, 0), 3)
        cv2.imshow('Camera', image)
        cv2.waitKey(30)
        
        

def main(args=None):
    rclpy.init(args=args)
    cam_to_num_publisher = CamToNumPublisher()
    rclpy.spin(cam_to_num_publisher)
    cam_to_num_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
