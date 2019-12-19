from socket import *
import RPi.GPIO as GPIO
import time
import os
import subprocess
serverPort = 12000
serverSocket = socket(AF_INET, SOCK_DGRAM)

# Assigns the server port to the server's pocket
serverSocket.bind(('', serverPort))

serverAddress = ('192.168.50.103', serverPort)
print("Beginning on %s port %s" % serverAddress)

print("The server is ready to receive!")
while True:
    # Packet arrives from the client and the message & src address is stored
    # The message converts the bytes to string type and capitalizes them
    # Then converts back to bytes and sends the message back to server's pocket
    message, clientAddress = serverSocket.recvfrom(2048)
    d = int(message.decode())

   # print("Received %s bytes from %s" % (len(message), clientAddress))
    #servoPIN = 17
    #GPIO.setmode(GPIO.BCM)
    #GPIO.setup(servoPIN, GPIO.OUT)
    if d>30:
        d = int(message.decode())
       # proc = subprocess.Popen(['python3', 'servo.py'], shell = False)
       # time.sleep(2.8)
       # proc.terminate()
   # print(d)
        proc = subprocess.Popen(['python3','pitch2.py'], shell = False)
       # proc2 = subprocess.Popen(['pyhton3', 'claw2.py'], shell = False)
       # process = proc1 + proc2;
        time.sleep(10)
        proc.terminate()
        proc = subprocess.Popen(['python3', 'pitch1.py'], shell = False)
        time.sleep(2.8)
        proc.terminate()
        proc = subprocess.Popen(['python3', 'pitch21.py'], shell = False)
        time.sleep(3)
        proc.terminate()


        proc = subprocess.Popen(['python3', 'gripper.py'], shell = False)
        time.sleep(1.5)
        proc.terminate()

