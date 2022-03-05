#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


# initialization
if __name__ == '__main__':

        # setup ros node
        rospy.init_node('jetbot_test')
        ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

        move = 0.0
        stop = 1.0


        # reverse 
        speed_l = -0.2
        speed_r = -0.2
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(1.0)

        # forward
        speed_l = 0.2
        speed_r = 0.2
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(1.0)


        # right
        speed_l = 0.2
        speed_r = -0.2
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(1.0)

        # left
        speed_l = -0.2
        speed_r = 0.2
        for i in range(10):
            msg = Float32MultiArray()
            msg.data = [move, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(0.1)

        msg.data = [stop, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(1.0)


