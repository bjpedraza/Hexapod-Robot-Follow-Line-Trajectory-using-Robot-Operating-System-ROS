#! /usr/bin/python3
# File name   : move.py
# Description : Controlling all servos
# Website	 : www.adeept.com
# E-mail	  : support@adeept.com
# Author	  : William
# Date		: 2019/04/08
import time
import Adafruit_PCA9685
from mpu6050 import mpu6050
import Kalman_filter
import PID
import threading
import RPIservo

'''
change this variables to 0 to reverse all the servos.
'''
set_direction = 1

'''
change these two variables to reverse the direction of the legs.
'''
if set_direction:
	leftSide_direction  = 1
	rightSide_direction = 0
else:
	leftSide_direction  = 0
	rightSide_direction = 1

'''
change these two variables to reverse the height of the legs.
'''
if set_direction:
	leftSide_height  = 0
	rightSide_height = 1
else:
	leftSide_height  = 1
	rightSide_height = 0

'''
change this variable to set the range of the height range.
'''
height_change = 35

'''
change these variable to adjuest the steady function.
'''
steady_range_Min = -40
steady_range_Max = 130
range_Mid = (steady_range_Min+steady_range_Max)/2
X_fix_output = range_Mid
Y_fix_output = range_Mid
steady_X_set = 73

'''
Set PID
'''
P = 5
I = 0.01
D = 0

'''
>>> instantiation <<<
'''
X_pid = PID.PID()
X_pid.SetKp(P)
X_pid.SetKd(I)
X_pid.SetKi(D)
Y_pid = PID.PID()
Y_pid.SetKp(P)
Y_pid.SetKd(I)
Y_pid.SetKi(D)
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)

try:
	sensor = mpu6050(0x68)
	mpu6050_connection = 1
except:
	mpu6050_connection = 0

'''
change these two variable to adjuest the steady status.
	   (X+)
	   /|\
  (Y+)<-+->(Y-)
	    |
	   (X-)
Example: If you want the forhead of the robot to point down,
         you need to increase the value target_X.
'''
target_X = 0
target_Y = 0


'''
Set a default pwm value for all servos.
'''
for i in range(0,16):
	exec('pwm%d=RPIservo.init_pwm%d'%(i,i))

'''
Get raw data from mpu6050.
'''
def mpu6050Test():
	while 1:
		accelerometer_data = sensor.get_accel_data()
		print('X=%f,Y=%f,Z=%f'%(accelerometer_data['x'],accelerometer_data['y'],accelerometer_data['x']))
		time.sleep(0.3)

		
def init_all():

	pwm.set_pwm(0, 0, 275)
	pwm.set_pwm(1, 0, 300)
	pwm.set_pwm(2, 0, 375)
	pwm.set_pwm(3, 0, 375)

	pwm.set_pwm(4, 0, 340)
	pwm.set_pwm(5, 0, 275)
	pwm.set_pwm(6, 0, 295)
	pwm.set_pwm(7, 0, 220)

	pwm.set_pwm(8, 0, 300)
	pwm.set_pwm(9, 0, 240)
	pwm.set_pwm(10, 0, 340)
	pwm.set_pwm(11, 0, 280)

	#pwm.set_pwm(12, 0, 300)
	#pwm.set_pwm(13, 0, 315)
	pwm.set_pwm(14, 0, pwm14)
	pwm.set_pwm(15, 0, pwm15)
	

init_all()

def ctrl_range(raw, max_genout, min_genout):
	if raw > max_genout:
		raw_output = max_genout
	elif raw < min_genout:
		raw_output = min_genout
	else:
		raw_output = raw
	return int(raw_output)

'''
left_I   -<forward>-- right_III
left_II  ---<BODY>---  right_II
left_III -<Backward>-   right_I

            pos=1
           /     \
          /       \
         /         \
    pos=2---pos=3---pos=4

Change the value of wiggle to set the range and direction that the legs moves.
'''

def left_I(pos,wiggle,heightAdjust=0):
	startHorPoint = 275+20
	startVerPoint = 300-30

	if pos == 0:
		#pwm.set_pwm(0,0,pwm0)
		if leftSide_height:
			pwm.set_pwm(1,0,startVerPoint+heightAdjust)
			time.sleep(0.2)
		else:
			time.sleep(0.2)
			pwm.set_pwm(1,0,startVerPoint-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(0,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(0,0,startHorPoint+wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					time.sleep(0.2)
					pwm.set_pwm(1,0,startVerPoint+height_change)
			elif pos == 3:
				pwm.set_pwm(0,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(0,0,startHorPoint-wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint+height_change)
					time.sleep(0.2)
		else:
			if pos == 1:
				pwm.set_pwm(0,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint+3*wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint-3*wiggle)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(0,0,startHorPoint-wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint+wiggle)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(0,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint+wiggle)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(0,0,startHorPoint+wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(1,0,startVerPoint+wiggle)
					time.sleep(0.2)


def left_II(pos,wiggle,heightAdjust=0):
	startHorPoint = 375-15
	startVerPoint = 375-30

	if pos == 0:
		#pwm.set_pwm(2,0,pwm2)
		if leftSide_height:
			pwm.set_pwm(3,0,startVerPoint+heightAdjust)
		else:
			pwm.set_pwm(3,0,startVerPoint-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(2,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(2,0,startHorPoint+wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(2,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(2,0,startHorPoint-wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint+height_change)
					time.sleep(0.2)
		else:
			if pos == 1:
				pwm.set_pwm(2,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint+3*wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint-3*wiggle)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(2,0,startHorPoint-wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint+wiggle)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(2,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint+wiggle)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(2,0,startHorPoint+wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(3,0,startVerPoint+wiggle)
					time.sleep(0.2)


def left_III(pos,wiggle,heightAdjust=0):
	startHorPoint = 340-20
	startVerPoint = 275-30

	if pos == 0:
		#pwm.set_pwm(4,0,pwm4)
		if leftSide_height:
			pwm.set_pwm(5,0,startVerPoint+heightAdjust)
		else:
			pwm.set_pwm(5,0,startVerPoint-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(4,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(4,0,startHorPoint+wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(4,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(4,0,startHorPoint-wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint+height_change)
					time.sleep(0.2)
		else:
			if pos == 1:
				pwm.set_pwm(4,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint+3*wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint-3*wiggle)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(4,0,startHorPoint-wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint+wiggle)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(4,0,startHorPoint)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint+wiggle)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(4,0,startHorPoint+wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,startVerPoint-wiggle)
					time.sleep(0.2)
				else:
					pwm.set_pwm(5,0,startVerPoint+wiggle)
					time.sleep(0.2)


def right_I(pos,wiggle,heightAdjust=0):
	#wiggle = -wiggle
	startHorPoint = 295+20
	startVerPoint = 255+10
	
	if pos == 0:
		#pwm.set_pwm(6,0,pwm6)
		if rightSide_height:
			pwm.set_pwm(7,0,startVerPoint+heightAdjust)
		else:
			pwm.set_pwm(7,0,startVerPoint-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(6,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(6,0,startHorPoint+wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(6,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(6,0,startHorPoint-wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint+height_change)
					time.sleep(0.2)
		else:
			if pos == 1:
				pwm.set_pwm(6,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(6,0,startHorPoint-wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(6,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(6,0,startHorPoint+wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(7,0,startVerPoint+height_change)
					time.sleep(0.2)


def right_II(pos,wiggle,heightAdjust=0):
	#wiggle = -wiggle
	startHorPoint = 300
	startVerPoint = 240+30

	if pos == 0:
		#pwm.set_pwm(8,0,pwm8)
		if rightSide_height:
			pwm.set_pwm(9,0,startVerPoint+heightAdjust)
		else:
			pwm.set_pwm(9,0,startVerPoint-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(8,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(8,0,startHorPoint+wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(8,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(8,0,startHorPoint-wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint+height_change)
					time.sleep(0.2)
		else:
			if pos == 1:
				pwm.set_pwm(8,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(8,0,startHorPoint-wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(8,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(8,0,startHorPoint+wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(9,0,startVerPoint+height_change)
					time.sleep(0.2)


def right_III(pos,wiggle,heightAdjust=0):
	#wiggle = -wiggle
	startHorPoint = 340-20
	startVerPoint = 280+30

	if pos == 0:
		#pwm.set_pwm(10,0,pwm10)
		if rightSide_height:
			pwm.set_pwm(11,0,startVerPoint+heightAdjust)
		else:
			pwm.set_pwm(11,0,startVerPoint-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(10,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(10,0,startHorPoint+wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(10,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(10,0,startHorPoint-wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint+height_change)
					time.sleep(0.2)
		else:
			if pos == 1:
				pwm.set_pwm(10,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint+3*height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint-3*height_change)
					time.sleep(0.2)
			elif pos == 2:
				pwm.set_pwm(10,0,startHorPoint-wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 3:
				pwm.set_pwm(10,0,startHorPoint)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint+height_change)
					time.sleep(0.2)
			elif pos == 4:
				pwm.set_pwm(10,0,startHorPoint+wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,startVerPoint-height_change)
					time.sleep(0.2)
				else:
					pwm.set_pwm(11,0,startVerPoint+height_change)
					time.sleep(0.2)


def move(step_input, speed, command):
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	if speed == 0:
		return

	if command == 'no':
		right_I(step_I, 60, 0)
		left_II(step_I, 50, 0)
		right_III(step_I, 50, 0)

		left_I(step_II, 50, 0)
		right_II(step_II, 50, 0)
		left_III(step_II, 60, 0)
	elif command == 'left':
		right_I(step_I, 60, 0)
		left_II(step_I, -50, 0)
		right_III(step_I, 50, 0)

		left_I(step_II, -50, 0)
		right_II(step_II, 50, 0)
		left_III(step_II, -60, 0)
	elif command == 'right':
		right_I(step_I, -60, 0)
		left_II(step_I, 50, 0)
		right_III(step_I, -50, 0)

		left_I(step_II, 50, 0)
		right_II(step_II, -50, 0)
		left_III(step_II, 60, 0)


def stand():
	pwm.set_pwm(0, 0, 275)
	pwm.set_pwm(1, 0, 300)
	pwm.set_pwm(2, 0, 375)
	pwm.set_pwm(3, 0, 375)
	pwm.set_pwm(4, 0, 340)
	pwm.set_pwm(5, 0, 275)
	pwm.set_pwm(6, 0, 295)
	pwm.set_pwm(7, 0, 220)
	pwm.set_pwm(8, 0, 300)
	pwm.set_pwm(9, 0, 240)
	pwm.set_pwm(10, 0, 340)
	pwm.set_pwm(11, 0, 280)
'''
def neckMid():
	pwm.set_pwm(12, 0, 300)
	pwm.set_pwm(13, 0, 315)
'''
'''
---Dove---
making the servo moves smooth.
'''
def dove_Left_I(horizontal, vertical):
	if leftSide_direction:
		pwm.set_pwm(0,0,pwm0 + horizontal)
	else:
		pwm.set_pwm(0,0,pwm0 - horizontal)

	if leftSide_height:
		pwm.set_pwm(1,0,pwm1+vertical)
	else:
		pwm.set_pwm(1,0,pwm1-vertical)


def dove_Left_II(horizontal, vertical):
	if leftSide_direction:
		pwm.set_pwm(2,0,pwm2 + horizontal)
	else:
		pwm.set_pwm(2,0,pwm2 - horizontal)

	if leftSide_height:
		pwm.set_pwm(3,0,pwm3+vertical)
	else:
		pwm.set_pwm(3,0,pwm3-vertical)


def dove_Left_III(horizontal, vertical):
	if leftSide_direction:
		pwm.set_pwm(4,0,pwm4 + horizontal)
	else:
		pwm.set_pwm(4,0,pwm4 - horizontal)

	if leftSide_height:
		pwm.set_pwm(5,0,pwm5+vertical)
	else:
		pwm.set_pwm(5,0,pwm5-vertical)


def dove_Right_I(horizontal, vertical):
	if rightSide_direction:
		pwm.set_pwm(6,0,pwm6 + horizontal)
	else:
		pwm.set_pwm(6,0,pwm6 - horizontal)

	if rightSide_height:
		pwm.set_pwm(7,0,pwm7+vertical)
	else:
		pwm.set_pwm(7,0,pwm7-vertical)


def dove_Right_II(horizontal, vertical):
	if rightSide_direction:
		pwm.set_pwm(8,0,pwm8 + horizontal)
	else:
		pwm.set_pwm(8,0,pwm8 - horizontal)

	if rightSide_height:
		pwm.set_pwm(9,0,pwm9+vertical)
	else:
		pwm.set_pwm(9,0,pwm9-vertical)


def dove_Right_III(horizontal, vertical):
	if rightSide_direction:
		pwm.set_pwm(10,0,pwm10 + horizontal)
	else:
		pwm.set_pwm(10,0,pwm10 - horizontal)

	if rightSide_height:
		pwm.set_pwm(11,0,pwm11+vertical)
	else:
		pwm.set_pwm(11,0,pwm11-vertical)


def dove(step_input, speed, timeLast, dpi, command):
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	
	if speed > 0:
		if step_input == 1:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, 3*speed_II)
					dove_Right_II(-speed_I, 3*speed_II)
					dove_Left_III(-speed_I, 3*speed_II)

					dove_Right_I(speed_I, -10)
					dove_Left_II(speed_I, -10)
					dove_Right_III(speed_I, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, 3*speed_II)
					dove_Right_II(-speed_I, 3*speed_II)
					dove_Left_III(speed_I, 3*speed_II)

					dove_Right_I(speed_I, -10)
					dove_Left_II(-speed_I, -10)
					dove_Right_III(speed_I, -10)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, 3*speed_II)
					dove_Right_II(speed_I, 3*speed_II)
					dove_Left_III(-speed_I, 3*speed_II)

					dove_Right_I(-speed_I, -10)
					dove_Left_II(speed_I, -10)
					dove_Right_III(-speed_I, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break

		elif step_input == 2:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, 3*(speed-speed_II))
					dove_Right_II(speed_II, 3*(speed-speed_II))
					dove_Left_III(speed_II, 3*(speed-speed_II))

					dove_Right_I(-speed_II, -10)
					dove_Left_II(-speed_II, -10)
					dove_Right_III(-speed_II, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, 3*(speed-speed_II))
					dove_Right_II(speed_II, 3*(speed-speed_II))
					dove_Left_III(-speed_II, 3*(speed-speed_II))

					dove_Right_I(-speed_II, -10)
					dove_Left_II(speed_II, -10)
					dove_Right_III(-speed_II, -10)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, 3*(speed-speed_II))
					dove_Right_II(-speed_II, 3*(speed-speed_II))
					dove_Left_III(speed_II, 3*(speed-speed_II))

					dove_Right_I(speed_II, -10)
					dove_Left_II(-speed_II, -10)
					dove_Right_III(speed_II, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
		elif step_input == 3:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, -10)
					dove_Right_II(speed_I, -10)
					dove_Left_III(speed_I, -10)

					dove_Right_I(-speed_I, 3*speed_II)
					dove_Left_II(-speed_I, 3*speed_II)
					dove_Right_III(-speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, -10)
					dove_Right_II(speed_I, -10)
					dove_Left_III(-speed_I, -10)

					dove_Right_I(-speed_I, 3*speed_II)
					dove_Left_II(speed_I, 3*speed_II)
					dove_Right_III(-speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, -10)
					dove_Right_II(-speed_I, -10)
					dove_Left_III(speed_I, -10)

					dove_Right_I(speed_I, 3*speed_II)
					dove_Left_II(-speed_I, 3*speed_II)
					dove_Right_III(speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
		elif step_input == 4:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, -10)
					dove_Right_II(-speed_II, -10)
					dove_Left_III(-speed_II, -10)

					dove_Right_I(speed_II, 3*(speed-speed_II))
					dove_Left_II(speed_II, 3*(speed-speed_II))
					dove_Right_III(speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, -10)
					dove_Right_II(-speed_II, -10)
					dove_Left_III(speed_II, -10)

					dove_Right_I(speed_II, 3*(speed-speed_II))
					dove_Left_II(-speed_II, 3*(speed-speed_II))
					dove_Right_III(speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, -10)
					dove_Right_II(speed_II, -10)
					dove_Left_III(-speed_II, -10)

					dove_Right_I(-speed_II, 3*(speed-speed_II))
					dove_Left_II(speed_II, 3*(speed-speed_II))
					dove_Right_III(-speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
	else:
		speed = -speed
		if step_input == 1:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, 3*speed_II)
					dove_Right_II(speed_I, 3*speed_II)
					dove_Left_III(speed_I, 3*speed_II)

					dove_Right_I(-speed_I, -10)
					dove_Left_II(-speed_I, -10)
					dove_Right_III(-speed_I, -10)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 2:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, 3*(speed-speed_II))
					dove_Right_II(-speed_II, 3*(speed-speed_II))
					dove_Left_III(-speed_II, 3*(speed-speed_II))

					dove_Right_I(speed_II, -10)
					dove_Left_II(speed_II, -10)
					dove_Right_III(speed_II, -10)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 3:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, -10)
					dove_Right_II(-speed_I, -10)
					dove_Left_III(-speed_I, -10)

					dove_Right_I(speed_I, 3*speed_II)
					dove_Left_II(speed_I, 3*speed_II)
					dove_Right_III(speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 4:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, -10)
					dove_Right_II(speed_II, -10)
					dove_Left_III(speed_II, -10)

					dove_Right_I(-speed_II, 3*(speed-speed_II))
					dove_Left_II(-speed_II, 3*(speed-speed_II))
					dove_Right_III(-speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				else:
					pass


def steady_X():
	if leftSide_direction:
		pwm.set_pwm(0,0,pwm0+steady_X_set)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X_set)
	else:
		pwm.set_pwm(0,0,pwm0+steady_X_set)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X_set)

	if rightSide_direction:
		pwm.set_pwm(10,0,pwm10+steady_X_set)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6-steady_X_set)
	else:
		pwm.set_pwm(10,0,pwm10-steady_X_set)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6+steady_X_set)


def steady():
	global X_fix_output, Y_fix_output
	if mpu6050_connection:
		accelerometer_data = sensor.get_accel_data()
		X = accelerometer_data['x']
		X = kalman_filter_X.kalman(X)
		Y = accelerometer_data['y']
		Y = kalman_filter_Y.kalman(Y)

		X_fix_output += -X_pid.GenOut(X - target_X)
		X_fix_output = ctrl_range(X_fix_output, steady_range_Max, -steady_range_Max)

		Y_fix_output += -Y_pid.GenOut(Y - target_Y)
		Y_fix_output = ctrl_range(Y_fix_output, steady_range_Max, -steady_range_Max)

		'''
		LEFT_I
		'''	
		left_I_input = ctrl_range((X_fix_output + Y_fix_output), steady_range_Max, steady_range_Min)
		left_I(0, 35, left_I_input)

		'''
		LEFT_II
		'''
		left_II_input = ctrl_range((abs(X_fix_output*0.5)+Y_fix_output), steady_range_Max, steady_range_Min)
		left_II(0, 35, left_II_input)

		'''
		LEFT_III
		'''
		left_III_input = ctrl_range((-X_fix_output + Y_fix_output), steady_range_Max, steady_range_Min)
		left_III(0, 35, left_III_input)

		'''
		RIGHT_III
		'''
		right_III_input = ctrl_range((X_fix_output - Y_fix_output), steady_range_Max, steady_range_Min)
		right_III(0, 35, right_III_input)

		'''
		RIGHT_II
		'''
		right_II_input = ctrl_range((abs(-X_fix_output*0.5)-Y_fix_output), steady_range_Max, steady_range_Min)
		right_II(0, 35, right_II_input)

		'''
		RIGHT_I
		'''
		right_I_input = ctrl_range((-X_fix_output-Y_fix_output), steady_range_Max, steady_range_Min)
		right_I(0, 35, right_I_input)



def steadyTest():
	if leftSide_direction:
		pwm.set_pwm(0,0,pwm0+steady_X)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X)
	else:
		pwm.set_pwm(0,0,pwm0+steady_X)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X)

	if rightSide_direction:
		pwm.set_pwm(10,0,pwm10+steady_X)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6-steady_X)
	else:
		pwm.set_pwm(10,0,pwm10-steady_X)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6+steady_X)

	while 1:
		left_H = steady_range_Min
		right_H = steady_range_Max
		left_I(0, 35, left_H)
		left_II(0, 35, left_H)
		left_III(0, 35, left_H)
		
		right_I(0, 35, right_H)
		right_II(0, 35, right_H)
		right_III(0, 35, right_H)

		time.sleep(1)
		
		left_H = 130
		right_H = -40
		left_I(0, 35, left_H)
		left_II(0, 35, left_H)
		left_III(0, 35, left_H)
		
		right_I(0, 35, right_H)
		right_II(0, 35, right_H)
		right_III(0, 35, right_H)

		time.sleep(1)
		


def relesae():
	pwm.set_all_pwm(0,0)


def clean_all():
	pwm.set_all_pwm(0, 0)


def destroy():
	clean_all()


SmoothMode = 0
steadyMode = 0

step_set = 1
speed_set = 100
DPI = 17

new_frame = 0
direction_command = 'no'
turn_command = 'no'


def move_thread():
	global step_set
	stand_stu = 1
	if not steadyMode:
		if direction_command == 'forward' and turn_command == 'no':
			if SmoothMode:
				dove(step_set,35,0.001,DPI,'no')
				step_set += 1
				if step_set == 5:
					step_set = 1

			else:
				move(step_set, 35, 'no')
				time.sleep(0.1)
				step_set += 1
				if step_set == 5:
					step_set = 1

		elif direction_command == 'backward' and turn_command == 'no':
			if SmoothMode:
				dove(step_set,-35,0.001,DPI,'no')
				step_set += 1
				if step_set == 5:
					step_set = 1

			else:
				move(step_set, -35, 'no')
				time.sleep(0.1)
				step_set += 1
				if step_set == 5:
					step_set = 1

		else:
			pass

		if turn_command != 'no':
			if SmoothMode:
				dove(step_set,35,0.001,DPI,turn_command)
				step_set += 1
				if step_set == 5:
					step_set = 1

			else:
				move(step_set, 35, turn_command)
				time.sleep(0.1)
				step_set += 1
				if step_set == 5:
					step_set = 1

		else:
			pass

		if turn_command == 'no' and direction_command == 'stand':
			stand()
			step_set = 1
		pass
	else:
		steady_X()
		steady()


class RobotM(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(RobotM, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

	def pause(self):
		#print('......................pause..........................')
		self.__flag.clear()

	def resume(self):
		self.__flag.set()

	def run(self):
		while 1:
			self.__flag.wait()
			move_thread()
			pass

rm = RobotM()
rm.start()
rm.pause()

def commandInput(command_input):
	global direction_command, turn_command, SmoothMode, steadyMode
	if 'forward' == command_input:
		direction_command = 'forward'
		rm.resume()
	
	elif 'backward' == command_input:
		direction_command = 'backward'
		rm.resume()

	elif 'stand' in command_input:
		direction_command = 'stand'
		rm.pause()


	elif 'left' == command_input:
		turn_command = 'left'
		rm.resume()

	elif 'right' == command_input:
		turn_command = 'right'
		rm.resume()

	elif 'no' in command_input:
		turn_command = 'no'
		rm.pause()

	elif 'automaticOff' == command_input:
		SmoothMode = 0
		steadyMode = 0
		rm.pause()

	elif 'automatic' == command_input:
		rm.resume()
		SmoothMode = 1

	elif 'KD' == command_input:
		steadyMode = 1
		rm.resume()

	elif 'speech' == command_input:
		steadyMode = 1
		rm.resume()

	elif 'speechOff' == command_input:
		SmoothMode = 0
		steadyMode = 0
		rm.pause()