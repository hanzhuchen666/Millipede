from math import sin, cos
from lx16a import *
import time
import keyboard

PI = 3.1415826


# LX16A.initialize("/dev/ttyUSB0", 0.1)
LX16A.initialize("COM4",0.1)
n_servo = 8
stands = [500, 500,
    500, 500,
    500, 500,
    500, 500]

criticalFrames = [
    [95.00, 166.6, 165.0, 78.56, 85.00, 155.32, 150.00, 94.20],
    [136.48, 126.56, 164.16, 98.48, 119.00, 110.52, 153.80, 110.04],
    [151.48, 111.56, 104.16, 138.48, 129.00, 105.52, 88.80, 155.04],
    [171.48, 86.56, 94.16, 148.48, 154.00, 85.52, 73.80, 165.04],
    [171.48, 111.56, 134.16, 108.48, 149.00, 110.52, 118.80, 120.04],
    [96.48, 166.56, 149.16, 88.48, 84.00, 155.52, 143.80, 100.04]

    ]

t = 0

try:
    # get the servos objects
    servos = []
    for i in range(n_servo):
        servos.append(LX16A(i+1))

    # get the servo positions limit
    posLimits = []
    for i in range(n_servo):
        posLimits.append(servos[i].get_angle_limits())

    # get the current position
    curPos = []
    for i in range(n_servo):
        curPos.append(servos[i].get_physical_angle())
    
    # get the middle points to stand
    for i in range(n_servo):
        stands[i] = (posLimits[i][1] - posLimits[i][0])/2 + posLimits[i][0]

    print(stands)
    



except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

def  Zeros():
    while True:
        for i in range(n_servo):
            servos[i].move(stands[i])
            curPos[i]= stands[i]
        time.sleep(0.01)

        for i in range(n_servo):
            if abs(servos[i].get_physical_angle() - stands[i]) >=0.3:
                continue
        break
    print("Zeros finished")
    time.sleep(2)

def moveToFrame(index):
    while True:
        for i in range(n_servo):
            servos[i].move(criticalFrames[index][i])
            curPos[i] = criticalFrames[index][i]
        time.sleep(0.05)
        for i in range(n_servo):
            if abs(servos[i].get_physical_angle() - criticalFrames[index][i]) >=0.3:
                continue
        break
    print("Frames Ready to Start")
    time.sleep(2)

    
def step1(time):
    # curPos[0] = (1+sin(0.5*t)) * (posLimits[0][1] - posLimits[0][0])/2 + posLimits[0][0]
    curPos[1] = (cos(0.5*t)+1) * (posLimits[1][1] - posLimits[1][0])/2 + posLimits[1][0]
    # curPos[2] = (1+sin(0.5*t)) * (posLimits[2][1] - posLimits[2][0])/2 + posLimits[2][0]
    curPos[3] = (cos(0.5*t)+1) * (posLimits[3][1] - posLimits[3][0])/2 + posLimits[3][0]
    # curPos[4] = (1+sin(0.5*t)) * (posLimits[4][1] - posLimits[4][0])/2 + posLimits[4][0]
    curPos[5] = (cos(0.5*t)+1) * (posLimits[5][1] - posLimits[5][0])/2 + posLimits[5][0]
    # curPos[6] = (1+sin(0.5*t)) * (posLimits[6][1] - posLimits[6][0])/2 + posLimits[6][0]
    curPos[7] = (cos(0.5*t)+1) * (posLimits[7][1] - posLimits[7][0])/2 + posLimits[7][0]

    
def step2(time):
    # curPos[0] = (1+sin(0.5*t)) * (posLimits[0][1] - posLimits[0][0])/2 + posLimits[0][0]
    curPos[1] = (cos(0.5*t)+1) * (posLimits[1][1] - posLimits[1][0])/2 + posLimits[1][0]
    # curPos[2] = (1+sin(0.5*t)) * (posLimits[2][1] - posLimits[2][0])/2 + posLimits[2][0]
    curPos[3] = (cos(0.5*t)+1) * (posLimits[3][1] - posLimits[3][0])/2 + posLimits[3][0]
    # curPos[4] = (1+sin(0.5*t)) * (posLimits[4][1] - posLimits[4][0])/2 + posLimits[4][0]
    curPos[5] = (cos(0.5*t)+1) * (posLimits[5][1] - posLimits[5][0])/2 + posLimits[5][0]
    # curPos[6] = (1+sin(0.5*t)) * (posLimits[6][1] - posLimits[6][0])/2 + posLimits[6][0]
    curPos[7] = (cos(0.5*t)+1) * (posLimits[7][1] - posLimits[7][0])/2 + posLimits[7][0]

def step3(time):
    # curPos[0] = (1+sin(0.5*t)) * (posLimits[0][1] - posLimits[0][0])/2 + posLimits[0][0]
    curPos[1] = (cos(0.5*t)+1) * (posLimits[1][1] - posLimits[1][0])/2 + posLimits[1][0]
    # curPos[2] = (1+sin(0.5*t)) * (posLimits[2][1] - posLimits[2][0])/2 + posLimits[2][0]
    curPos[3] = (cos(0.5*t)+1) * (posLimits[3][1] - posLimits[3][0])/2 + posLimits[3][0]
    # curPos[4] = (1+sin(0.5*t)) * (posLimits[4][1] - posLimits[4][0])/2 + posLimits[4][0]
    curPos[5] = (cos(0.5*t)+1) * (posLimits[5][1] - posLimits[5][0])/2 + posLimits[5][0]
    # curPos[6] = (1+sin(0.5*t)) * (posLimits[6][1] - posLimits[6][0])/2 + posLimits[6][0]
    curPos[7] = (cos(0.5*t)+1) * (posLimits[7][1] - posLimits[7][0])/2 + posLimits[7][0]

def step4(time):
    # curPos[0] = (1+sin(0.5*t)) * (posLimits[0][1] - posLimits[0][0])/2 + posLimits[0][0]
    curPos[1] = (cos(0.5*t)+1) * (posLimits[1][1] - posLimits[1][0])/2 + posLimits[1][0]
    # curPos[2] = (1+sin(0.5*t)) * (posLimits[2][1] - posLimits[2][0])/2 + posLimits[2][0]
    curPos[3] = (cos(0.5*t)+1) * (posLimits[3][1] - posLimits[3][0])/2 + posLimits[3][0]
    # curPos[4] = (1+sin(0.5*t)) * (posLimits[4][1] - posLimits[4][0])/2 + posLimits[4][0]
    curPos[5] = (cos(0.5*t)+1) * (posLimits[5][1] - posLimits[5][0])/2 + posLimits[5][0]
    # curPos[6] = (1+sin(0.5*t)) * (posLimits[6][1] - posLimits[6][0])/2 + posLimits[6][0]
    curPos[7] = (cos(0.5*t)+1) * (posLimits[7][1] - posLimits[7][0])/2 + posLimits[7][0]


debugIndex = 0

def keyboardDebug(x):
    global debugIndex
    print(x.scan_code)

    if x.event_type=='down' and x.scan_code == 75 and debugIndex >0:
        debugIndex -= 1
    elif x.event_type=='down' and x.scan_code==77 and debugIndex <7:
        debugIndex += 1
    elif x.event_type=='down' and x.scan_code==80 and curPos[debugIndex]>posLimits[debugIndex][0]+5:
        curPos[debugIndex] -= 5
        print(curPos[debugIndex])
    elif x.event_type=="down" and x.scan_code==72 and curPos[debugIndex]<posLimits[debugIndex][1] -5:
        curPos[debugIndex] += 5
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==2:
        debugIndex = 0
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==3:
        debugIndex = 1
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==4:
        debugIndex = 2
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==5:
        debugIndex = 3
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==6:
        debugIndex = 4
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==7:
        debugIndex = 5
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==8:
        debugIndex = 6
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==9:
        debugIndex = 7
        print(curPos[debugIndex])
    elif x.event_type=='down' and x.scan_code==25:
        print("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]"%(curPos[0], curPos[1], curPos[2], curPos[3], curPos[4], curPos[5], curPos[6], curPos[7]))
    elif x.event_type=='down' and x.scan_code==44:
        moveToFrame(0)
    elif x.event_type=='down' and x.scan_code==45:
        moveToFrame(1)
    elif x.event_type=='down' and x.scan_code==46:
        moveToFrame(2)
    elif x.event_type=='down' and x.scan_code==47:
        moveToFrame(3)
    elif x.event_type=='down' and x.scan_code==48:
        moveToFrame(4)
    elif x.event_type=='down' and x.scan_code==49:
        moveToFrame(5)
       

    
def cosTo(T, t0, fromFrame:int, toFrame:int):
    global criticalFrames, curPos
    for i in range(8):
        curPos[i] = (1 - cos(PI*(t-t0)/T))*(criticalFrames[toFrame][i] - criticalFrames[fromFrame][i])/2 +criticalFrames[fromFrame][i]



keyboard.hook(keyboardDebug)


# Zeros()
moveToFrame(0)
while True:

    T = 1
    step = t//T
    print(step)
    fromFrame = int(step%len(criticalFrames))
    toFrame = int((fromFrame+1)%len(criticalFrames))
    cosTo(T, step*T, fromFrame, toFrame)



    # here to input the next target movement
    # curPos[0] = (1+sin(0.5*t)) * (posLimits[0][1] - posLimits[0][0])/2 + posLimits[0][0]
    # curPos[1] = (cos(0.5*t)+1) * (posLimits[1][1] - posLimits[1][0])/2 + posLimits[1][0]
    # # cur1Pos[2] = (1+sin(0.5*t)) * (posLimits[2][1] - posLimits[2][0])/2 + posLimits[2][0]
    # curPos[3] = (cos(0.5*t)+1) * (posLimits[3][1] - posLimits[3][0])/2 + posLimits[3][0]
    # # curPos[4] = (1+sin(0.5*t)) * (posLimits[4][1] - posLimits[4][0])/2 + posLimits[4][0]
    # curPos[5] = (cos(0.5*t)+1) * (posLimits[5][1] - posLimits[5][0])/2 + posLimits[5][0]
    # # curPos[6] = (1+sin(0.5*t)) * (posLimits[6][1] - posLimits[6][0])/2 + posLimits[6][0]
    # curPos[7] = (cos(0.5*t)+1) * (posLimits[7][1] - posLimits[7][0])/2 + posLimits[7][0]


    ## here to make the target position stay in limitations
    for i in range(n_servo):
        if curPos[i] < posLimits[i][0]:
            curPos[i] = posLimits[i][0] + 0.5
        elif curPos[i] > posLimits[i][1]:
            curPos[i] = posLimits[i][1] - 0.5

    ## move the servos
    for i in range(n_servo):
        servos[i].move(curPos[i])

    ## print things
    # print(servos[0].get_physical_angle())    
    # print(servos[0].get_commanded_angle())
    # print(servos[0].get_waiting_angle())




    time.sleep(0.01)
    t += 0.05


