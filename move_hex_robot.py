#! /usr/bin/python3
# File name   : move_hex_robot.py
# Description : Controlling all servos

import time
from mpu6050 import mpu6050
import rospy
from std_msgs.msg import String

import robot_functions as rf

step = 1

def callback(data):
        #print(data.data)
        myMsg = data.data
        
        if step == 1:
	        turn_command = myMsg
	elif step == 2:
		turn_command = myMsg
	elif step == 3:
		turn_command = myMsg
	elif step == 4:
		#rf.leftSide_direction  = 0
		#rf.rightSide_direction = 1
		turn_command = myMsg
	elif step == 5:
		stand()
	
	rf.move(step, 35, turn_command)
	
	step += 1
	if step > 4:
		step = 1
	time.sleep(0.08)


def listener():

	# name for our 'listener'
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("lineDec", String, callback)

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	try:
		while 1:
			#Subscriber:
			listener()

		'''
		while 1:
			dove(1,-35,0.01,17,'no')
			dove(2,-35,0.01,17,'no')
			dove(3,-35,0.01,17,'no')
			dove(4,-35,0.01,17,'no')
            
		#mpu6050Test()
		#print(sensor.get_temp())
		
		steady_X()
		while 1:
			steady()
			time.sleep(0.02)

		for i in range(0,9):
			look_left()
			time.sleep(1)
		for i in range(0,16):
			look_right()
			time.sleep(1)	
		time.sleep(1)
		look_home()
		'''
		#rf.pwm.set_all_pwm(0,0)
		#rf.pwm.set_all_pwm(0, 300)
		#time.sleep(10)
        
	except KeyboardInterrupt:
		rf.pwm.set_all_pwm(0, 300)
		time.sleep(1)
		#rf.clean_all()

