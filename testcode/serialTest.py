from time  import sleep
import RPi.GPIO as GPIO
import serial
import struct
import rclpy
                
def main(args=None):
    global cnt
    pin = 17
    ser = serial.Serial("/dev/ttyACM0", 9600)

    while True: 
        ser.write(struct.pack('B', 80))
        sleep(1)

if __name__ == '__main__':
    main()