import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import cv2
import numpy as np
from picamera2 import Picamera2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from my_py_interface.msg import Lidar

# 이미지에서 검출된 후보 좌표값 반환
def get_candidate(image, width, height):
    # BGR 이미지를 HSV로 변환
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 블러링으로 이미지를 부드럽게 변환
    blurred_image = cv2.medianBlur(hsv_image, 3)
    kernel = np.ones((5, 5), np.uint8)
    # 이미지를 침식하여 노이즈 제거
    blurred_image = cv2.erode(blurred_image, kernel, iterations=1)
    # 검은색 범위를 정의 (Hue 0-180, Saturation 0-255, Value 0-38)
    black_mask = cv2.inRange(blurred_image, (0, 0, 0), (180, 255, 38))
    res = [0]
    i = 0
    # 이미지의 너비를 따라 검은색 픽셀을 찾음
    while i < width:
        j = 0
        # 연속된 검은색 픽셀을 찾음
        while i + j < width and (black_mask[height-1][i+j] == 255 or black_mask[height-10][i+j] == 255):
            j += 1
        if j > 3:  # 연속된 검은색 픽셀이 4개 이상일 때만 중앙점을 계산
            res.append(i + j // 2)
        i += j + 1
    res += [width]
    return res

# 후보 좌표 사이의 오차를 계산
def get_error(pl1, pl2):
    return abs(pl1[0] - pl2[0]) + abs(pl1[1] - pl2[1])

# 현재 x 좌표 반환
def get_now_x(image, width, height, prev_x):
    candidate = get_candidate(image, width, height)
    now_x = candidate[:2]
    min_sum_error = get_error(now_x, prev_x)
    # 후보 좌표 중에서 이전 좌표와 가장 오차가 작은 좌표를 선택
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
        # 카메라 설정 및 시작
        self.camConfig = self.picam2.create_preview_configuration(main={"size": (self.width, self.height)}, controls={"FrameDurationLimits": (16666, 16666)})
        self.picam2.configure(self.camConfig)
        self.picam2.start()

        # 퍼블리셔 초기화
        self.publisher = self.create_publisher(Int16MultiArray, '/cam_to_num', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.prev_x = [0, self.width]
        self.now_x = [0, self.width]
        self.msg = Int16MultiArray()
        
        self.publisher_ = self.create_publisher(CompressedImage, '/current_img', 10)
        self.timer2 = self.create_timer(timer_period, self.image_callback)
        self.timer3 = self.create_timer(timer_period, self.change_x)

        self.socket = 'None'
        self.subscription_socket = self.create_subscription(
            String,
            '/line_change',
            self.socket_callback,
            10
        )
        self.subscription_socket
        
        self.lane_changing = 0
        
        self.subscription_lidar = self.create_subscription(
            Lidar,
            '/object_detection',
            self.lidar_callback,
            10
        )
        self.block_left = False
        self.block_right = False
        
    # 라이다 데이터를 처리하여 장애물 정보를 업데이트
    def lidar_callback(self, msg):
        if self.block_left != msg.left:
            self.get_logger().info(f'lidar_left_signal : {msg.left}')

        if self.block_right != msg.right:
            self.get_logger().info(f'lidar_right_signal : {msg.right}')

        self.block_left = msg.left
        self.block_right = msg.right
        
    # 소켓 데이터를 처리하여 차선 변경 신호를 업데이트
    def socket_callback(self, msg):
        if self.socket == 'None':
            self.socket = msg.data
        self.get_logger().info(f'line_change_signal : {self.socket}')

    # 차선 변경을 처리
    def change_x(self):
        if self.socket == 'left' and not self.block_left:
            self.prev_x = [0, self.prev_x[0]]
            self.now_x = [self.prev_x[0], self.prev_x[1]]
            self.lane_changing = 1
            self.socket = 'None'
        elif self.socket == 'right' and not self.block_right:
            self.prev_x = [self.prev_x[1], 640]
            self.now_x = [self.prev_x[0], self.prev_x[1]]
            self.lane_changing = 1
            self.socket = 'None'
        
    # 주기적으로 카메라 이미지를 처리하여 현재 좌표를 계산하고 publish
    def timer_callback(self):
        image = self.picam2.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        self.prev_x = self.now_x
        self.now_x = get_now_x(image, self.width, self.height, self.prev_x)
        
        if self.now_x[0] != 0 and self.now_x[1] != 640:
            self.lane_changing = 0
        
        self.msg.data = self.now_x + [self.lane_changing]
        self.publisher.publish(self.msg)
        cv2.line(image, (self.now_x[0], self.height-1), (self.now_x[0], self.height-10), (0, 255, 0), 3)        
        cv2.line(image, (self.now_x[1], self.height-1), (self.now_x[1], self.height-10), (0, 255, 0), 3)
        cv2.imshow('Camera', image)
        cv2.waitKey(30)
    
    # 웹에서 모니터링하기 위해 주기적으로 이미지를 캡처하여 publish
    def image_callback(self):
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.line(frame, (self.now_x[0], self.height-1), (self.now_x[0], self.height-10), (0, 255, 0), 3)        
        cv2.line(frame, (self.now_x[1], self.height-1), (self.now_x[1], self.height-10), (0, 255, 0), 3)
        # OpenCV를 사용하여 이미지를 JPEG 포맷으로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            image_message = CompressedImage()
            image_message.format = "jpeg"
            image_message.data = np.array(buffer).tobytes()
            self.publisher_.publish(image_message)
    
    # 노드를 종료할 때 호출되는 함수
    def destroy_node(self):
        self.picam2.release()
        super().destroy_node()

def main(args=None):
    # ROS 2 초기화 및 노드 실행
    rclpy.init(args=args)
    cam_to_num_publisher = CamToNumPublisher()
    rclpy.spin(cam_to_num_publisher)
    cam_to_num_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()