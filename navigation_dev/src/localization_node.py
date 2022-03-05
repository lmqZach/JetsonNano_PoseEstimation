#!/usr/bin/env python

import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np


pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)


def tag_callback(msg):
    # TODO: implement localization logic
    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    if msg.ids:
        tag_id = msg.ids[0]
        tm = msg.detections[0].matrix
        tm = np.array(tm).reshape(4, 4)
        origin = np.array([[0, 0, 0, 1]]).reshape(4, 1)
        relative_pos = np.matmul(tm, origin).reshape(4)
        relative_x, relative_y = relative_pos[0], relative_pos[2]
        pose_msg.pose.matrix = [relative_x, relative_y, tag_id]
        # TODO
        # pose_msg.pose = [R11, R12, R13, t1,
        #                  R21, R22, R23, t2,
        #                  R31, R32, R33, t3]
    else:
        pose_msg.pose.matrix = [0, 0, -1]
    pose_pub.publish(pose_msg)


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
