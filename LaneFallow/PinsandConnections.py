import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2


#Pins
GPIO.setmode(GPIO.BCM)# I set with broadcom
GPIO.setwarnings(False)

#Servo pin for front wheels
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)#Speed control for servo
servo_pwm.start(0)

#Raspberry pi DC motors pins
MOTOR_AIN1 = 23
MOTOR_AIN2 = 24
MOTOR_PWMA = 12
MOTOR_BIN1 = 25
MOTOR_BIN2 = 26
MOTOR_PWMB = 13
STBY_PIN = 5
#Definition GPIO out pins
GPIO.setup(MOTOR_AIN1, GPIO.OUT)
GPIO.setup(MOTOR_AIN2, GPIO.OUT)
GPIO.setup(MOTOR_PWMA, GPIO.OUT)
GPIO.setup(MOTOR_BIN1, GPIO.OUT)
GPIO.setup(MOTOR_BIN2, GPIO.OUT)
GPIO.setup(MOTOR_PWMB, GPIO.OUT)
GPIO.setup(STBY_PIN, GPIO.OUT)
#get motor driver on 
GPIO.output(STBY_PIN, GPIO.HIGH)

pwm_a = GPIO.PWM(MOTOR_PWMA, 500)
pwm_b = GPIO.PWM(MOTOR_PWMB, 500)
pwm_a.start(0)
pwm_b.start(0)

#set servo for stability
def set_servo_angle(angle):
    if angle < 55: angle = 55#hold servo certein values because when servo turn too much wheel is crashes into vehicle
    if angle > 125: angle = 125
    duty = angle / 18 + 2
    servo_pwm.ChangeDutyCycle(duty)

def initialize_camera():
    
    picam2 = Picamera2()#class and connection with cam
    camera_config = picam2.create_preview_configuration( #preview to cam
    main={"size": (640, 480)},
    sensor={'output_size': (2328, 1748), 'bit_depth': 10}#size and colordepth
    )
    picam2.configure(camera_config)
    picam2.start()#start to cam
    return picam2

#back side motors just go forward
def move_robot(speed=50):
    if speed > 100: speed = 100 #speed limit
    if speed < 0: speed = 0 #speedlimit
    GPIO.output(MOTOR_AIN1, GPIO.HIGH)
    GPIO.output(MOTOR_AIN2, GPIO.LOW)
    GPIO.output(MOTOR_BIN1, GPIO.HIGH)
    GPIO.output(MOTOR_BIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def stop_motors():
    GPIO.output(MOTOR_AIN1, GPIO.LOW)
    GPIO.output(MOTOR_AIN2, GPIO.LOW)
    GPIO.output(MOTOR_BIN1, GPIO.LOW)
    GPIO.output(MOTOR_BIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

def cleanup_PinsandConnections(picam2,servo_pwm,pwm_a, pwm_b):
    stop_motors()
    servo_pwm.stop()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()

