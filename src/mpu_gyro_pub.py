#! /usr/bin/python3
from mpu6050 import mpu6050
import rospy
from std_msgs.msg import String #or Float32

try:
        sensor = mpu6050(0x68)
        mpu6050_connection = 1
except:
        mpu6050_connection = 0


def talker():
        pub = rospy.Publisher('Axis', String, queue_size=20)
        rospy.init_node('talker', anonymous=True)
        #print(mpu6050_connection)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
                accelerometer_data = str( sensor.get_accel_data() )
                #print('X=%f,Y=%f,Z=%f'%(accelerometer_data['x'],accelerometer_data['y'],accelerometer_data['z']))
                rospy.loginfo('sent')
                pub.publish(accelerometer_data)
                rate.sleep()


if __name__ == '__main__':
        try:
                talker()
        except rospy.ROSInterruptException:
                pass