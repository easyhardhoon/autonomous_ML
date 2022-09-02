#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
from types import NoneType
import pycuda.autoinit
import time
from util.yolo_classes import get_cls_dict
from util.display import show_fps
from util.yolo_with_plugins import TrtYOLO
from my_msgs.msg import yolo

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


image = np.empty(shape=[0])
bridge = CvBridge()
yolo_motor = None
img_ready = False

CAM_FPS = 30
WIDTH, HEIGHT = 640, 480


def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

def yolo_send(num):
    global yolo_motor
    yolo_motor_msg = yolo()
    yolo_motor_msg.yolo_state = num
    yolo_motor.publish(yolo_motor_msg)

category_num = 2
model = 'yolov4_new'
letter_box = False
conf_th = 0.2
cls_dict = get_cls_dict(category_num)
trt_yolo = TrtYOLO(model, category_num, letter_box)

if not os.path.isfile('/home/nvidia/xycar_ws/src/yolo_node/yolo/%s.trt' % model):
    raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % model)


def detect_yolo(img):
    boxes, confs, clss = trt_yolo.detect(img, conf_th)    
    return boxes, confs, clss  

yolo_status_c = []
for i in range(5):
	yolo_status_c.append(0)

yolo_status_o = []
for i in range(5):
	yolo_status_o.append(0)

th_status_c = []
for i in range(5):
	th_status_c.append(0)

th_status_o = []
for i in range(5):
	th_status_o.append(0)

def is_crosswalk(boxes,confs,clss):
    global yolo_status_c
    global is_walk
    if(is_walk):
        return
    ccc = False
    index = 0
    tmp = 0
    if(clss.size!=0):
        for c in clss:
            if c == 0:
                ccc = True
                tmp = index
            index+=1
    if(boxes.size!=0 and ccc and confs[tmp] > 0.3): 
        yolo_status_c.pop(0)
        yolo_status_c.append(1)
    else:
        yolo_status_c.pop(0)
        yolo_status_c.append(0)
    if(sum(yolo_status_c) >=3):
        is_walk = True
        return
    is_walk = False
    return

def is_obstacle(boxes,clss):
    global yolo_status_o
    global is_obs
    if(is_obs):
        return
    if(boxes.size!=0 and sum(clss) >0):
        yolo_status_o.pop(0)
        yolo_status_o.append(1)
    else:
        yolo_status_o.pop(0)
        yolo_status_o.append(0)
    if(sum(yolo_status_o) >=3):
        is_obs = True
        return
    is_obs = False
    return
		
def th_crosswalk(boxes,clss,stop_th_y):
    global th_status_c
    ccc = 0
    index = 0
    if(clss.size!=0):
        for c in clss:
            if(c ==0):
                ccc = index
            index+=1
    if(boxes.size !=0 and boxes[ccc,3] > stop_th_y):
        th_status_c.pop(0)
        th_status_c.append(1)
    else:
        th_status_c.pop(0)
        th_status_c.append(0)
    if(sum(th_status_c) >=3):
        return True
    return False

def th_obstacle(boxes,stop_th_y):
    global th_status_o
    tmp = 0
    index = 0
    check = 0
    if(boxes.size!=0):
        for box in boxes:
            if(box[3] > tmp):
                tmp = box[3]
                check = index
            index+=1
    if(boxes.size !=0 and boxes[check,3] > stop_th_y):
        th_status_o.pop(0)
        th_status_o.append(1)
    else:
        th_status_o.pop(0)
        th_status_o.append(0)
    if(sum(th_status_o) >=3):
        return True
    return False

def start_cross_walk(img):
    global is_walk, yolo_send_state
    boxes,confs,clss = detect_yolo(img)
    is_crosswalk(boxes,confs,clss)
    if(is_walk):
        if(th_crosswalk(boxes,clss,327)): # final__tuning
            yolo_send_state = 3
        else:
            yolo_send_state = 2
        is_walk = False


def start_obstacle(img):
    global is_obs, yolo_send_state
    boxes,confs,clss = detect_yolo(img)
    is_obstacle(boxes,clss)
    if(is_obs and th_obstacle(boxes,300)): # lidar ~ 60cm 's pixel boundary "300"
        yolo_send_state = 1
    else:
        yolo_send_state = 0

def main():
    global image, img_ready, yolo_motor, yolo_send_state
    global th_status_c,th_status_o, yolo_status_c, yolo_status_o, is_walk, is_obs
    is_walk = False
    is_obs = False
    rospy.init_node('yolo_node')
    print("YOLO START!!!!!!")
    yolo_motor = rospy.Publisher('/Yolo', yolo, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    while not rospy.is_shutdown():
        while img_ready == False:
            continue
        img = image.copy()
        img_ready = False
        yolo_send_state = 0
        img = img[0:420,0:640] 
        img2 = img.copy()
        start_obstacle(img)
        start_cross_walk(img2)
        yolo_send(yolo_send_state)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
