import serial
import time

arduino = serial.Serial(port='COM9', baudrate=115200, timeout=.1)

x = '300v200/n'

while True:
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
