import rclpy
import RPi.GPIO as GPIO
import serial
import time

from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Int8, Int32
from my_py_interface.msg import Lidar

# 서보 모터 핀 설정
servoPin = 12
# 서보 모터의 최대 및 최소 듀티 설정
SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

# GPIO 모드 설정 및 서보 핀 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPin, GPIO.OUT)
servo = GPIO.PWM(servoPin, 50)  # PWM 신호 주파수 50Hz로 설정
servo.start(0)

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.pre_error = 0
        self.ser = serial.Serial("/dev/ttyACM0", 115200)  # 시리얼 통신 초기화
        self.signal = False
        # 왼쪽 0, 오른쪽 1을 나타내는 신호 정보
        self.signal_info = -1
        self.nearCar_Left = False
        self.nearCar_Right = False

        # 속도 제어 타이머 설정 (0.01초 간격)
        speed_timer_period = 0.01
        self.timer = self.create_timer(speed_timer_period, self.speed_callback)

        # 속도 퍼블리셔 초기화
        self.publisher = self.create_publisher(Int8, '/speed', 10)
        self.msg = Int32()
        self.temp = Int8()
        
        self.servo_angle = 90  # 서보 모터의 초기 각도 설정
        self.x1 = 0
        self.x2 = 640
        self.signal = -1
        self.signal_info = -1

        self.front = False
        self.back = False
        self.left = False
        self.right = False

        self.change_speed_signal = False
        self.change_speed_value = Int8()  # 속도 변경 신호 값

        # 카메라로부터 데이터 구독
        self.subscription_camera = self.create_subscription(
            Int16MultiArray,
            '/cam_to_num',
            self.camera_callback,
            10)
        self.subscription_camera

        # 라이다로부터 데이터 구독
        self.subscription_lidar = self.create_subscription(
            Lidar,
            '/object_detection',
            self.speed_control_lidar_callback,
            10
        )
        self.subscription_lidar
    
        # 소켓으로부터 속도 제어 신호 구독
        self.subscription_speed_socket = self.create_subscription(
            Int8,
            '/speed_control',
            self.speed_control_socket_callback,
            10
        )
        self.subscription_speed_socket

        # 차선 변경 신호 구독
        self.subscription_changeLine_socket = self.create_subscription(
            String,
            '/line_change',
            self.socket_callback,
            10
        )
        self.subscription_changeLine_socket,

    def speed_callback(self):
        # 차선 변경 신호는 받았으나, 차선 변경이 불가능한 경우: 속도 감속
        if ((self.signal_info == 0 and self.left == True) or (self.signal_info == 1 and self.right == True)):
            self.temp.data = 6
        # 차선 변경 가능한 경우: 속도 감속
        elif self.signal == True:
            self.temp.data = 6
        # 차선 변경 신호가 없으며, servo_angle이 커브 정도를 반영해서 속도 감속
        else:
            # 차량 속도 변경
            if self.change_speed_signal == True:
                self.temp.data = max((7 + self.change_speed_value) - abs(self.servo_angle - 90) // 10, 6)
                self.change_speed_signal = False
            else:
                self.temp.data = max(7 - abs(self.servo_angle - 90) // 20, 5)

        # 차량 앞/뒤 존재여부에 따른 속도 조정
        if self.front == True: 
            self.temp.data = 2
        elif self.back == True:
            self.temp.data = 10
        
        self.publisher.publish(self.temp)  # 0.01초마다 속도 퍼블리시

    def speed_control_lidar_callback(self, msg):
        # 라이다 데이터 변화에 따른 로그 출력
        if self.front != msg.front:
            self.get_logger().info(f'front_lidar_change : {msg.front}')
        
        if self.back != msg.back:
            self.get_logger().info(f'back_lidar_change : {msg.back}')
    
        # 라이다 데이터를 바탕으로 전/후/좌/우 상태 업데이트
        self.front = msg.front
        self.back = msg.back
        self.left = msg.left
        self.right = msg.right

    def speed_control_socket_callback(self, msg):
        # 소켓을 통해 속도 변경 신호 수신
        self.change_speed_signal = True
        self.change_speed_value = msg.data 

    def camera_callback(self, msg):
        # 카메라 데이터를 바탕으로 차선 중심 계산
        self.x1, self.x2, self.signal = msg.data
        self.get_logger().info(f'x1: {self.x1}, x2: {self.x2}')

        pixel_min, pixel_max = 0, 640
        servo_min, servo_max = 60, 120
        
        if (self.x1 != 0 and self.x2 != 640) or (self.x1 == 0 and self.x2 == 640):
            center = (self.x1 + self.x2) / 2
        else:
            if self.x1 == 0:
                center = 0
            elif self.x2 == 640:
                center = 640
                
        error = 320 - center

        # 서보 모터 각도 계산 및 제한
        self.servo_angle = int(0.3 * error + 90)
        self.servo_angle = max(min(self.servo_angle, servo_max), servo_min)
        
        # 신호가 1인 경우 서보 각도 조정
        if self.signal == 1:
            self.servo_angle = (self.servo_angle - 90) * 0.5 + 90 
        
        # 서보 각도 시리얼 통신으로 전송
        self.ser.write(f'steering {self.servo_angle}\n'.encode('UTF-8'))
        print('angle : ', self.servo_angle)
        self.pre_error = error
        
        # 차선 변경 후 signal 초기화
        if self.signal == True and self.x1 != 0 and self.x2 != 640:
            self.signal = False
            self.signal_info = -1
            self.get_logger().info(f'line_change_signal : {self.signal}')

    def socket_callback(self, msg):
        # 차선 변경 신호 수신
        if msg.data == 'left':
            self.signal_info = 0
        else:
            self.signal_info = 1


def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)
    control_node = ControlNode()
    # 노드 스핀
    rclpy.spin(control_node)
    # 노드 종료
    control_node.destroy_node()
    servo.stop()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()