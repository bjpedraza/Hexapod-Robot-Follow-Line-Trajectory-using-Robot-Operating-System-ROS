#! /usr/bin/python3
import cv2
import math
import numpy as np
import time
import rospy
from std_msgs.msg import String
import Adafruit_PCA9685

cap = cv2.VideoCapture(0)
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

cameraXMid = 319.5
cameraYMid = 179.5
cameraXRight = cameraXMid + 8
cameraXLeft = cameraXMid - 8
cameraYUp = cameraYMid - 8
cameraYDown = cameraYMid + 8


'''
change this variables to 0 to reverse all the servos.
'''
set_direction = 1

'''
change these two variables to adjuest the function for observing.
'''
if set_direction:
	Up_Down_direction = 1
	Left_Right_direction = 1
else:
	Up_Down_direction = 0
	Left_Right_direction = 0

Left_Right_input = 300
Up_Down_input = 315

Left_Right_Max = 375
Left_Right_Min = 205
Up_Down_Max = 400
Up_Down_Min = 260

look_wiggle = 30
move_stu = 1


def init_all():
	pwm.set_pwm(12, 0, 300)
	pwm.set_pwm(13, 0, 235)

init_all()

def neckMid():
	pwm.set_pwm(12, 0, 300)
	pwm.set_pwm(13, 0, 235)

def look_up(wiggle=look_wiggle):
	global Up_Down_input
	if Up_Down_direction:
		Up_Down_input += wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
	else:
		Up_Down_input -= wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
	pwm.set_pwm(13, 0, Up_Down_input)
	return Up_Down_input


def look_down(wiggle=look_wiggle):
	global Up_Down_input
	if Up_Down_direction:
		Up_Down_input -= wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
	else:
		Up_Down_input += wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
	pwm.set_pwm(13, 0, Up_Down_input)
	return Up_Down_input


def look_left(wiggle=look_wiggle):
	global Left_Right_input
	if Left_Right_direction:
		Left_Right_input += wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
	else:
		Left_Right_input -= wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
	pwm.set_pwm(12, 0, Left_Right_input)
	return Left_Right_input


def look_right(wiggle=look_wiggle):
	global Left_Right_input
	if Left_Right_direction:
		Left_Right_input -= wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
	else:
		Left_Right_input += wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
	pwm.set_pwm(12, 0, Left_Right_input)
	return Left_Right_input


def look_home():
	global Left_Right_input, Up_Down_input
	pwm.set_pwm(13, 0, 300)
	pwm.set_pwm(12, 0, 300)
	Left_Right_input = 300
	Up_Down_input = 300


neckHor = 300
neckVer = 235
neckStep = 1

def neckMove(speed):
	global neckStep
	global neckHor
	global neckVer

	if neckStep == 1:
		neckVer = look_up(speed)
	elif neckStep == 2:
		neckVer = look_down(speed)
	elif neckStep == 3:
		neckHor = look_left(speed)
	elif neckStep == 4:
		neckHor = look_right(speed)
	elif neckStep == 5:
		neckMid()

	if neckVer >= 400:
		neckStep = 2
	elif neckVer <= 260 and neckStep==2:
		neckStep = 3
		neckMid()
	elif neckHor > 374:
		neckStep = 4
	elif neckHor < 259:
		neckStep = 5

def neckSend(neckH):
    if(neckH < 290):
        myMsg = 'right'
    elif(neckH > 310):
        myMsg = 'left'
    else:
        myMsg = 'straight'

    return myMsg


def talker():
    global neckStep
    global neckHor
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('lineDec', String, queue_size=50)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        ret, src = cap.read()
        closesPoint = -1
        closesLine = []

        src = cv2.resize(src, (640, 360))
        dst = cv2.Canny(src, 50, 200, None, 3)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 30, None, 30, 10)
        #data = []
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                if(linesP[i][0][1] > closesPoint or linesP[i][0][3] > closesPoint):
                	closesLine = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
        
        print('here is lineP:',linesP)
        print(closesLine)
        if closesLine.any():
            if(closesLine[0] > cameraXLeft and closesLine[0] < cameraXRight):
                myMsg = 'no'
            elif(closesLine[0] < cameraXLeft):
                myMsg = 'left'
                #look_left(2.5)
            elif(closesLine[0] > cameraXRight):
                myMsg = 'right'
                #look_right(2.5)

            time.sleep(0.3)

        #d = linesP.tolist()
        #for x in range(len(d)
        #print(len(d[0][0]))
        #d=[[int(d[i][0][k]) for k in range(len(d[0][0]))] for i in range(len(d))]
        #print(d)

        cv2.imshow("Detected Lines", cdstP)
        #cv2.imshow("Source", src)
        rospy.loginfo('sent')
        pub.publish(myMsg)
        rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        cap.release()
        cv2.destroyAllWindows()
        pass
