import machine
import time

M1A = machine.Pin(16, machine.Pin.OUT)
M1B = machine.Pin(17, machine.Pin.OUT)
M2A = machine.Pin(18, machine.Pin.OUT)
M2B = machine.Pin(19, machine.Pin.OUT)

encoderRight = machine.Pin(23, machine.Pin.IN)
encoderLeft = machine.Pin(22, machine.Pin.IN)

m1a = machine.PWM(M1A)
m1b = machine.PWM(M1B)
m2a = machine.PWM(M2A)
m2b = machine.PWM(M2B)

posR = 0
posL = 0

speedRight = 900.0
speedLeft = 900.0

Kp = 0.1

while True:
    
    m1a.duty(int(speedLeft))
    m1b.duty(0)
    m2a.duty(int(speedRight))
    m2b.duty(0)

    prevR = encoderRight.value()
    prevL = encoderLeft.value()
    time.sleep(0.01)
    currR = encoderRight.value()
    currL = encoderLeft.value()
    
    if(currR != prevR):
        posR = posR + 1
        
    if(currL != prevL):
        posL = posL + 1
    
    error = posR-posL
    
    if error>0: #right side is ahead, speed must inc on left
        speedLeft = speedLeft + Kp*error
        speedRight = 900
    if error<0:
        speedRight = speedRight + Kp*abs(error)
        speedLeft = 900 
        
    print(error, speedLeft, speedRight)
    time.sleep(0.01)
        
        
    
