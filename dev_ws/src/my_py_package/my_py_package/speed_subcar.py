import time
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import RPi.GPIO as GPIO
import serial
import struct
import rclpy

from std_msgs.msg import Int32

GPIOpin = 17
cnt = 0
isDetected = False

class SteeringSubscriber(Node):
    def __init__(self):
        super().__init__('steering_subscriber')
        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.prev_time = str(time.time()).split('.')[0]
        self.prev_error = 0
        self.drive = 100
        
        self.subscription_steering = self.create_subscription(
            Int16MultiArray,
            '/steering',
            self.steering_callback,
            10)
        self.subscription_steering
    
    def steering_callback(self, msg):
        global cnt
        print(f'detect_count: {cnt}')
        angle = msg.data

        #current_time = str(time.time()).split('.')[0]
        
        #target_cnt = 6
        
        #different_time = int(current_time) - int(self.prev_time)
        
        if 78 < angle < 92:
            # if different_time >= 1:
            #     error = target_cnt - cnt/different_time
                
            #     self.drive = min(max(int(self.drive+1*error+1*(error-self.prev_error)/different_time), 0), 255)
                
            self.ser.write(f'drive {190}\n'.encode('UTF-8'))
                # self.get_logger().info(f'detect_count: {cnt}, speed: {self.drive}')
                # #print(target_cnt, self.drive, cnt, different_time, error, self.prev_error)
                # self.prev_error = error
                # cnt = 0
        else:
            self.ser.write(f'drive {240}\n'.encode('UTF-8'))

        #detectMetal()
        self.prev_time = current_time

    

def initialInductive():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIOpin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def detectMetal():
    global cnt
    global isDetected

    if(GPIOpin != -1):
        state = GPIO.input(GPIOpin)
        if not state:
            if isDetected == False:
                cnt += 1
                isDetected = True
        else:
            isDetected = False
    else:
        print("please initial input pin")
        
                
def main(args=None):
    initialInductive()
    
    rclpy.init(args=args)
    steering_subsciber = SteeringSubscriber()
    rclpy.spin(steering_subsciber)

    steering_subsciber.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    