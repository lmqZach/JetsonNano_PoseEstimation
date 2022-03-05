#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy




# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
        max_pwm = 115.0
        speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

        if motor_ID == 1:
                motor = motor_left
                ina = 1
                inb = 0
        elif motor_ID == 2:
                motor = motor_right
                ina = 2
                inb = 3
        else:
                rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
                return
        
        motor.setSpeed(speed)

        if value > 0:
                motor.run(Adafruit_MotorHAT.FORWARD)
                motor_driver._pwm.setPWM(ina,0,speed*16)
                motor_driver._pwm.setPWM(inb,0,0)
        else:
                motor.run(Adafruit_MotorHAT.BACKWARD)
                motor_driver._pwm.setPWM(ina,0,0)
                motor_driver._pwm.setPWM(inb,0,speed*16)
                                


# stops all motors
def all_stop():
        motor_left.setSpeed(0)
        motor_right.setSpeed(0)

        motor_left.run(Adafruit_MotorHAT.RELEASE)
        motor_right.run(Adafruit_MotorHAT.RELEASE)
        
        motor_driver._pwm.setPWM(0,0,0)
        motor_driver._pwm.setPWM(1,0,0)
        motor_driver._pwm.setPWM(2,0,0)
        motor_driver._pwm.setPWM(3,0,0)

# directional commands (degree, speed)
def on_cmd_dir(msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

        if msg.data.lower() == "left":
                set_speed(motor_left_ID,  -1.0)
                set_speed(motor_right_ID,  1.0) 
        elif msg.data.lower() == "right":
                set_speed(motor_left_ID,   1.0)
                set_speed(motor_right_ID, -1.0) 
        elif msg.data.lower() == "forward":
                set_speed(motor_left_ID,   1.0)
                set_speed(motor_right_ID,  1.0)
        elif msg.data.lower() == "backward":
                set_speed(motor_left_ID,  -1.0)
                set_speed(motor_right_ID, -1.0)  
        elif msg.data.lower() == "stop":
                all_stop()
        else:
                rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)

def joystick_callback(msg):

        speed = msg.axes[3] 
        
        # backward 
        if msg.axes[7] == 1:
            set_speed(motor_left_ID,  -speed)
            set_speed(motor_right_ID, -speed)  

        # forward
        elif msg.axes[7] == -1:
            set_speed(motor_left_ID,   speed)
            set_speed(motor_right_ID,  speed)
        # right
        elif msg.axes[6] == 1:
            set_speed(motor_left_ID,   speed)
            set_speed(motor_right_ID, -speed) 
        # left
        elif msg.axes[6] == -1:
            set_speed(motor_left_ID,  -speed)
            set_speed(motor_right_ID,  speed) 
        # stop
        else:
            cmd="stop"
            all_stop()

def ctrl_callback(msg):

        
        command = msg.data[0] 
        speed_l = -msg.data[1]
        speed_r = -msg.data[2]
        
        # move 
        if command == 0:
            set_speed(motor_left_ID,  speed_l)
            set_speed(motor_right_ID, speed_r)  

        else:
            cmd="stop"
            all_stop()

# initialization
if __name__ == '__main__':

        # setup motor controller
        motor_driver = Adafruit_MotorHAT(i2c_bus=1)

        motor_left_ID = 1
        motor_right_ID = 2

        motor_left = motor_driver.getMotor(motor_left_ID)
        motor_right = motor_driver.getMotor(motor_right_ID)

        # stop the motors as precaution
        all_stop()

        # setup ros node
        rospy.init_node('jetbot_control')
        
        rospy.Subscriber('/joy', Joy, joystick_callback)
        rospy.Subscriber('/ctrl_cmd', Float32MultiArray, ctrl_callback)

        # start running
        rospy.spin()

        # stop motors before exiting
        all_stop()

