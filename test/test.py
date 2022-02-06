#!/usr/bin/env python

import rospy
from pointsmap_renderer.srv import *
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

import cv2
import numpy as np

def rendering_client():
    camera_info = CameraInfo()
    camera_info.header.frame_id = 'camera'
    camera_info.height = 256
    camera_info.width = 512
    camera_info.K = [256.0, 0.0, 256.0, 0.0, 128.0, 256.0, 0.0, 0.0, 1.0]
    bridge = CvBridge()
    rospy.wait_for_service('rendering')
    try:
        rendering = rospy.ServiceProxy('rendering', CameraInfo2Image)
        res = rendering(camera_info)
        cv_image = bridge.imgmsg_to_cv2(res.sparse_depth, desired_encoding='passthrough')
        depth = np.where(cv_image > 80.0, 65535.0, cv_image / 80.0 * 65535.0).astype(np.uint16)
        cv2.imwrite('test.png', depth)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rendering_client()
