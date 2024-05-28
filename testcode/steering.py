import RPi.GPIO as GPIO
from time import sleep
import serial
import struct

servoPin          = 12
SERVO_MAX_DUTY    = 12
SERVO_MIN_DUTY    = 3

GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)
servo.start(0)


def setServoPos(degree):
  if degree > 180:
    degree = 180

  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
  print("Degree: {} to {}(Duty)".format(degree, duty))
  GPIO.setup(servoPin, GPIO.OUT)
  servo.ChangeDutyCycle(duty)
  sleep(0.3)
  GPIO.setup(servoPin, GPIO.IN)



if __name__ == "__main__":
  ser = serial.Serial("/dev/ttyACM0", 115200)

  for pos in range(118, 157, 10):
      setServoPos(pos)
      ser.write(struct.pack('B', pos))
      sleep(1)

  for pos in range(156, 119, -10):
      setServoPos(pos)
      ser.write(struct.pack('B', pos))
      sleep(1)

  for pos in range(118, 79, -10):
      setServoPos(pos)
      ser.write(struct.pack('B', pos))
      sleep(1)

  for pos in range(80, 119 , 10):
      setServoPos(pos)
      ser.write(struct.pack('B', pos))
      sleep(1)

  servo.stop()
  GPIO.cleanup()