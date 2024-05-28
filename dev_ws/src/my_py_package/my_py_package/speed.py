import time
import RPi.GPIO as GPIO
import serial
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int8

# GPIO 핀 번호 설정
GPIOpin = 17
# 금속 감지 카운터 초기화
cnt = 0
# 금속 감지 상태 초기화
isDetected = False

class SteeringSubscriber(Node):
    def __init__(self):
        super().__init__('steering_subscriber')
        # 시리얼 통신 초기화 (시리얼 포트와 보드레이트 설정)
        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        # 이전 시간 초기화
        self.prev_time = str(time.time()).split('.')[0]
        # 이전 오류 값 초기화
        self.prev_error = 0
        # 초기 구동 속도 설정
        self.drive = 100
        
        # 속도 명령 구독 설정
        self.subscription_steering = self.create_subscription(
            Int8,
            '/speed',
            self.steering_callback,
            10)
        self.subscription_steering
    
    def steering_callback(self, msg):
        global cnt
        
        # 현재 시간을 초 단위로 얻음
        current_time = str(time.time()).split('.')[0]
        # 목표 카운트 값 설정
        target_cnt = msg.data
        # 시간 차이 계산
        different_time = int(current_time) - int(self.prev_time)

        if different_time >= 1:
            # 오차 계산
            error = target_cnt - cnt / different_time
            # PD 제어를 통해 속도 조정
            # 비례 제어 (P): 5 * error
            # 미분 제어 (D): 5 * (error - self.prev_error) / different_time
            # 적분 제어 (I)는 사용하지 않음
            self.drive = min(max(int(self.drive + 5 * error + 5 * (error - self.prev_error) / different_time), 0), 255)
            
            # 시리얼 통신을 통해 새로운 속도 전송
            self.ser.write(f'drive {self.drive}\n'.encode('UTF-8'))
            self.get_logger().info(f'detect_count: {cnt}, speed: {self.drive}')
            print(target_cnt, self.drive, cnt, different_time, error, self.prev_error)
            # 카운트 초기화
            cnt = 0

        detectMetal()
        # 이전 시간 및 오차 업데이트
        self.prev_time = current_time
        self.prev_error = error
    

def initialInductive():
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BCM)
    # GPIO 핀을 입력으로 설정하고 풀다운 저항 사용
    GPIO.setup(GPIOpin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def detectMetal():
    global cnt
    global isDetected

    if GPIOpin != -1:
        state = GPIO.input(GPIOpin)
        if not state:
            if isDetected == False:
                cnt += 1
                isDetected = True
        else:
            isDetected = False
    else:
        print("please initialize input pin")
        
                
def main(args=None):
    # 금속 감지 초기화
    initialInductive()
    
    # ROS 2 초기화
    rclpy.init(args=args)
    # SteeringSubscriber 노드 생성
    steering_subscriber = SteeringSubscriber()
    # 노드를 스핀하여 콜백 함수를 계속 호출할 수 있도록 함
    rclpy.spin(steering_subscriber)

    # 노드 소멸
    steering_subscriber.destroy_node()
    # GPIO 핀 초기화
    GPIO.cleanup()
    # ROS 2 종료
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
