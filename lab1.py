# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 15:48:24 2021

@author: maxma
"""

import numpy as np
import cv2
  
import sys
import RPi.GPIO as GPIO
from time import sleep

LED_PIN = 23
SERVO_PIN - 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwn.start(0)

def SetAngle(angle):
    duty=angle/18+2
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(SERVO_PIN,False)
    pwm.ChangeDutyCycle(0)

cap = cv2.VideoCapture(0)

while(True):
    ret, captured_frame = cap.read()
    output_frame = captured_frame.copy()

    
    captured_frame_bgr = cv2.cvtColor(captured_frame, cv2.COLOR_BGRA2BGR)
    captured_frame_bgr = cv2.medianBlur(captured_frame_bgr, 3)
    captured_frame_lab = cv2.cvtColor(captured_frame_bgr, cv2.COLOR_BGR2Lab)
    captured_frame_lab_red = cv2.inRange(captured_frame_lab, np.array([20, 150, 150]), np.array([190, 255, 255]))
    captured_frame_lab_red = cv2.GaussianBlur(captured_frame_lab_red, (5, 5), 2, 2)
    circles = cv2.HoughCircles(captured_frame_lab_red, cv2.HOUGH_GRADIENT, 1, captured_frame_lab_red.shape[0] / 8, param1=100, param2=18, minRadius=5, maxRadius=60)
	if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        cv2.circle(output_frame, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(0, 255, 0), thickness=2)
        print(circles)
        if (circles[0, 0] > 350) :
           print ('move+')
        elif (circles[0, 0] < 250) :
           print ('move-')
    cv2.imshow('frame', output_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()