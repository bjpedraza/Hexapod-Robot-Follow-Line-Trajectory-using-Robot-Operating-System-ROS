#! /usr/bin/python3
# File name   : move_hex_robot.py
# Description : Controlling all servos

from mpu6050 import mpu6050
import rospy
from std_msgs.msg import String

import robot_functions as rf

step = 1
myMsg = 'no'

def callback(data):
        rospy.loginfo("I heard %s",data.data)
	myMsg = data.data

def walk():
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
	else:
		print("step out of bound")

	if step == 5 or myMsg == '' : # Special case where step is 5 or empty msg the robot stands still
		rf.stand()
	else:
		rf.move(step, 35, turn_command)
		#rf.dove(step,-35, 0.01, 17, turn_command)
	
	step += 1
	if step > 4:
		step = 1


def listener():
	# name for our 'listener'
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("lineDec", String, callback)


if __name__ == '__main__'():

	listener()
	try:
		while not rospy.is_shutdown():
        		rospy.sleep(rospy.Duration(nsecs=1000)) # gives time to process msg
        		walk()
        except rospy.ROSInterruptException:
		print("shutdown")
			

	''' --------------- OR ------------------------
		while 1:
			rf.dove(1,-35,0.01,17,'no')
			rf.dove(2,-35,0.01,17,'no')
			rf.dove(3,-35,0.01,17,'no')
			rf.dove(4,-35,0.01,17,'no')
            
		#mpu6050Test()
		#print(rf.sensor.get_temp())
		
		rf.steady_X()
		while 1:
			rf.steady()
			time.sleep(0.02)

		for i in range(0,9):
			rf.look_left()
			time.sleep(1)
		for i in range(0,16):
			rf.look_right()
			time.sleep(1)	
		time.sleep(1)
		rf.look_home()
		
		#rf.pwm.set_all_pwm(0,0)
		#rf.pwm.set_all_pwm(0, 300)
		#time.sleep(10)
        
	except KeyboardInterrupt:
		rf.pwm.set_all_pwm(0, 300)
		time.sleep(1)
		#rf.clean_all()
	'''

