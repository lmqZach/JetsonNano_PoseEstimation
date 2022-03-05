#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np

ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)


def pose_callback(msg):
    global visited, turn_flag
    move = 0.0
    stop = 1.0
    # TODO: estimate control actions
    cmd_msg = Float32MultiArray()
    pose_mat = np.array(msg.pose.matrix)
    relative_x, relative_y, tag_id = pose_mat[0], pose_mat[1], pose_mat[2]
    print(relative_x, relative_y, tag_id)
    if tag_id == 42:
        visited = True
    if visited and tag_id == 0:  # if we have seen 0 before and see it again, we back to origin
        if relative_x < -0.2:
            cmd_msg.data = [move, -0.2, 0]
        else:
            cmd_msg.data = [stop, 0, 0]
    else:
        if relative_y < 0.5:
            diff = 0.2
            print('turning!!!')
            if turn_flag:
                cmd_msg.data = [move, 0, diff]
            else:
                cmd_msg.data = [stop, 0, 0]
            turn_flag = not turn_flag
        else:
            diff = relative_x * 0.1
            forward = min(0.4, relative_y * 0.3)
            speed_l = forward + diff
            speed_r = forward - diff
            cmd_msg.data = [move, speed_l, speed_r]
    ctrl_pub.publish(cmd_msg)


if __name__ == "__main__":
    visited = False
    turn_flag = True
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    rospy.spin()
