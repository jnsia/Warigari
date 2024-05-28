import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int16
import serial

class CamToNumSubscriber(Node):
    def __init__(self):
        super().__init__('cam_to_num_subscriber')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/cam_to_num',
            self.listener_callback,
            10)
        self.pre_error = 0
        self.servo_angle_msg = Int16()
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial("/dev/ttyACM0", 115200)  # 시리얼 통신 시작
        self.publisher = self.create_publisher(Int16, '/steering',10)

    def listener_callback(self, msg):
        x1, x2 = msg.data
        print(x1, x2)
        pixel_min, pixel_max = 0, 640
        servo_min, servo_max = 47, 123
        center = (x1 + x2) / 2
        error = 320 - center
        servo_angle = int(0.3 * error + 80)
        servo_angle = max(min(servo_angle, servo_max), servo_min)

        self.ser.write(f'steering {servo_angle}\n'.encode('UTF-8'))  # 아두이노로 각도 조정 명령 전송
        #print('angle : ', servo_angle)
        self.pre_error = error
        self.servo_angle_msg.data = servo_angle
        self.publisher.publish(self.servo_angle_msg)
    
def main(args=None):
    rclpy.init(args=args)
    cam_to_num_subscriber = CamToNumSubscriber()
    rclpy.spin(cam_to_num_subscriber)
    cam_to_num_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
