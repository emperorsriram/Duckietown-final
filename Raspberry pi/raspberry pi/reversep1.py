import RPi.GPIO as GPIO
import time
from grab import grippe
from pitch_2 import p2
import  os
import subprocess
servoPIN = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 60) # GPIO 17 for PWM with 50Hz
p.start(10) # Initialization
a=0
servoOp1 = True
while servoOp1:
    print ("P!")
   # p.ChangeDutyCycle(10.5)
   # time.sleep(0.5)
    p.ChangeDutyCycle(11)
    time.sleep(0.5)
    p.ChangeDutyCycle(11.5)
    time.sleep(0.5)
    p.ChangeDutyCycle(12.5)
    time.sleep(0.5)

    cond = p2()
    if cond:
        print("calling grab")
        proc = subprocess.Popen(['python3','gripper.py'],shell =False)
        time.sleep(2)
        proc.terminate()
        servoOp1 = False
  # if cond:
       # servoOp1 = False
   # p.ChangeDutyCycle(12.5)
   # time.sleep(0.5)
   # p.ChangeDutyCycle(7.5)
   # time.sleep(0.5)
   # p.ChangeDutyCycle(5)
    #time.sleep(0.5)
    #p.ChangeDutyCycle(2.5)
    #time.sleep(0.5)
#p.ChangeDutyCycle(2.5)
p.stop()
GPIO.cleanup()

