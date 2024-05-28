import time
import RPi.GPIO as GPIO
import serial
import struct

GPIOpin = -1
cnt = 0
isDetected = False


def initialInductive(pin):
	global GPIOpin
	GPIOpin = pin
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(GPIOpin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	print(GPIOpin, "pin finished inition")

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

if __name__ == '__main__':
	pin = 17
	initialInductive(pin)
	ser = serial.Serial("/dev/ttyACM0", 115200)
	prev_time = str(time.time()).split('.')[0]
	while True:
		current_time = str(time.time()).split('.')[0]
		different_time = int(current_time) - int(prev_time)
		if different_time >= 1:
			speed = cnt * 5.5 * 3.14
			print("count per second", cnt)
			print("speed", speed, "(cm/s)")
			ser.write(struct.pack('B', int(speed)))
			cnt = 0
		detectMetal()
		prev_time = current_time

