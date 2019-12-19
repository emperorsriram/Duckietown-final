import RPi.GPIO as GPIO
import time

lServoPin = 11
rServoPin = 13
GPIO.setmode(GPIO.BOARD)

GPIO.setup(lServoPin, GPIO.OUT)
GPIO.setup(rServoPin, GPIO.OUT)

lPwm = GPIO.PWM(lServoPin, 50)
rPwm = GPIO.PWM(rServoPin, 50)
lPwm.start(5)
rPwm.start(5)


x=0    
while(x<1):

       # p.ChangeDutyCycle(2.5)
       # time.sleep(0.5)

       # positionl = 1./18.*(i)+2
       # positionr = 1./18.*(180-i)+2
    lPwm.ChangeDutyCycle(7.5)
    rPwm.ChangeDutyCycle(6)
    time.sleep(0.5)
    #for i in range(90, 45, -1):
       # positionl = 1./18.*(i)+2
     #   positionr = 1./18.*(180-i)+2
        #lPwm.ChangeDutyCycle(positionl)
      #  rPwm.ChangeDutyCycle(positionr)
       # time.sleep(0.005)
    x = x + 1

lPwm.stop()
rPwm.stop()
GPIO.cleanup()

