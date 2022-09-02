#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
from types import NoneType
import pycuda.autoinit
import time
from my_msgs.msg import mission_trigger

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

motor = None
done_motor = None
done_cross = False
crosswalk_stop_state = False
done_motor_msg = String()


def mt_callback(data):
    global crosswalk_stop_state, done_cross
    if(data.crosswalk):
        crosswalk_stop_state = True
    else:
        crosswalk_stop_state = False
        done_cross = False

def drive(angle, speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

def done_send(data):
    global done_motor, done_motor_msg
    # done_motor_msg = String()
    done_motor_msg.data = data
    done_motor.publish(done_motor_msg)

def main():
    global motor, done_motor, crosswalk_stop_state, done_cross
    rospy.init_node('cross_walk')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    done_motor = rospy.Publisher('/mission_name_done', String, queue_size=1)
    rospy.Subscriber("/mission_trigger", mission_trigger, mt_callback, queue_size=1 )

    while not rospy.is_shutdown():
        if done_cross is True:
            continue
        if crosswalk_stop_state:
            drive(0,0)
            time.sleep(7)
            done_send("crosswalk")
            crosswalk_stop_state = False
            done_cross = True
        #cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
