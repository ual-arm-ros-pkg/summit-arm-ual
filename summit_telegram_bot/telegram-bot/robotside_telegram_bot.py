#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------------
# Telegram Bot: script that runs on the robot and uploads status updates to
# the server.
#
# Jose Luis Blanco Claraco - Sep 2019
# -----------------------------------------------------------------------------

import rospy
import requests
import logging
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import TelegramBotCommonConstants as TBC
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError

# Enable logging
logging.basicConfig(format='%(asctime)s-%(name)s-%(levelname)s-%(message)s',
                    level=logging.INFO)
logger = logging.getLogger(__name__)

# -- Global vars --
TOPIC_CAMERA = '/camera'  # Type: sensor_msgs/Image
TOPIC_2D_LIDAR = '/scan'  # Type: sensor_msgs/LaserScan
TOPIC_BATTERY = '/battery'  # Type: std_msgs/Float32
TOPIC_ODOMETRY = '/odom'  # Type: nav_msgs/Odometry
DECIMATION = 200
# -----------------

myCvBridge = CvBridge()


def upload_to_server(filename):
    '''Uploads a file to the telegram bot server'''
    f = open(filename)
    r = requests.post(url='http://ingmec.ual.es/robot-summit/upload.php',
                      data={'title': 'file'},  files={'UPLOAD': f})
    if r.status_code != 200:
        logger.error('Error uploading file to server!')


def decimate_counters(caller):
    '''Decimate counters for all possible callers'''
    if caller not in decimate_counters.__dict__:
        decimate_counters.__dict__[caller] = 0
        return True
    decimate_counters.__dict__[caller] = decimate_counters.__dict__[caller] + 1
    if decimate_counters.__dict__[caller] < DECIMATION:
        return False
    else:
        decimate_counters.__dict__[caller] = 0
        return True


def callback_camera_img(data):
    '''Gets a new camera image'''
    if not decimate_counters("callback_camera_img"):
        return
    try:
        cv_image = myCvBridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    fil = '/tmp/' + TBC.FILENAME_CAMERA_IMG
    cv2.imwrite(fil, cv_image)
    upload_to_server(fil)


def callback_battery(data):
    '''Gets a new battery voltage measure'''
    if not decimate_counters("callback_battery"):
        return
    fil = '/tmp/' + TBC.FILENAME_BATTERY
    open(fil, 'w').write("Voltaje de batería: %.03fV" % data.data)
    upload_to_server(fil)


def callback_2d_lidar(data):
    '''Gets a new 2D lidar scan'''
    if not decimate_counters("callback_2d_lidar"):
        return
    rospy.loginfo(rospy.get_caller_id() + "I got a lidar scan")
    IMG_SIZE = 500
    IMG_SCALE = 80.0
    CXY = IMG_SIZE / 2
    SCAN_DECIM = 3
    SCAN_DECIM_COUNTER = SCAN_DECIM
    frame = np.ones((IMG_SIZE, IMG_SIZE, 3), np.uint8)
    frame = frame * 255
    angle = data.angle_min
    for r in data.ranges:
        SCAN_DECIM_COUNTER = SCAN_DECIM_COUNTER + 1
        if SCAN_DECIM_COUNTER < SCAN_DECIM:
            continue
        SCAN_DECIM_COUNTER = 0

        # change infinite values to 0
        if math.isinf(r) is True:
            r = 0

        a = angle + (-90.0 * 3.1416 / 180.0)
        x = CXY + math.trunc(r * IMG_SCALE * math.cos(a))
        y = CXY + math.trunc(r * IMG_SCALE * math.sin(a))

        if x < 2 or x >= IMG_SIZE - 2 or y < 2 or y >= IMG_SIZE - 2:
            continue

        cv2.line(frame, (CXY, CXY), (x, y), (200, 200, 200), 1)
        frame[(y - 1):(y + 1), (x - 1):(x + 1)] = (0, 0, 255)

        angle = angle + data.angle_increment

    cv2.circle(frame, (250, 250), 2, (255, 255, 0))
    fil = '/tmp/' + TBC.FILENAME_LIDAR
    cv2.imwrite(fil, frame)
    upload_to_server(fil)


def callback_odometry(data):
    '''Gets a new odometry'''
    if not decimate_counters("callback_odometry"):
        return
    fil = '/tmp/' + TBC.FILENAME_POSE
    open(fil, 'w').write(
        "<b>Pose según odometría</b>:\n " + str(data.pose.pose))
    upload_to_server(fil)


def main():
    # Anonymous=false since we only want ONE instance of this node:
    rospy.init_node('robotside_telegram_bot', anonymous=False)
    rospy.Subscriber(TOPIC_CAMERA, Image, callback_camera_img)
    rospy.Subscriber(TOPIC_2D_LIDAR, LaserScan, callback_2d_lidar)
    rospy.Subscriber(TOPIC_BATTERY, Float32, callback_battery)
    rospy.Subscriber(TOPIC_ODOMETRY, Odometry, callback_odometry)
    rospy.spin()


if __name__ == '__main__':
    main()
