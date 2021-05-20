#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import sys
import os
import signal

# 상태를 초기화해준다.
def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])  # 카메라 노드로 받아올 이미지를 초기화
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40
angle = 0  # 회전각
dist1 = None  #  ROI 이미지(아래쪽 노란 차선 인식)
dist2 = None  #  ROI 이미지(신호등의 빨간색 인식)
dist3 = None  #  ROI 이미지(신호등의 초록색 인식)
dist4 = None  #  ROI 이미지(마지막 신호등에서 오른쪽 영역의 초록색 인식)
dist5 = None  #  ROI 이미지(마지막 신호등에서 왼쪽 영역의 초록색 인식)


# change state funciton
def change_state(task):
    task_ = task
    print 'Task changed to [%s]' % task_
    return task_


# OpenCV를 사용해 camera 노드에서 image를 받아온다.
def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def callback(data):
    global ranges_l, ack_msg
    ranges_l = data.ranges


# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)


# xycar를 멈춘다.
def drive_out():
    global ack_msg, ack_publisher
    for stop_cnt in range(10):
        ack_msg.speed = 0
        ack_msg.angle = 0
        ack_publisher.publish(ack_msg)
        time.sleep(0.1) # Stop


# 차선 위에 라인을 그린다.
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img


# 양쪽 차선 위와 양쪽 차선 중간에 사각형을 그린다.
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img


# left lines, right lines을 나눈다.
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines


# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b


# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    # show image
    cv2.imshow('calibration', frame)

    return lpos, rpos


# HSV Color Space를 사용해 관심영역을 지정한다.
def roi_process(image, lpos, rpos):
    global Width
    global Offset, Gap
    global image
    # hsv yellow
    lowerb1 = (20, 100, 100)
    upperb1 = (25, 150, 150)
    # hsv red
    lowerb2 = (0, 70, 70)
    upperb2 = (5, 150, 150)
    # hsv green
    lowerb3 = (60, 70, 70)
    upperb3 = (80, 150, 150)

    # roi2 Under Region for yellow line 
    roi2 = image[Offset: Offset + Gap + 20, 0: Width]
    roi2 = cv2.cvtColor(roi2, cv2.COLOR_GRAY2BGR)
    roi2 = draw_rectangle(roi2, lpos, rpos)

    # roi3 Upper Region for 3 sinal lights
    roi3 = image[30: 150, 280: Width - 120]
    roi3 = cv2.cvtColor(roi3, cv2.COLOR_GRAY2BGR)
    roi3 = draw_rectangle(roi2, lpos, rpos)

    # roi4 Upper Right Reigion for 4 signal lights
    roi4 = image[0:200, 330 : 500]
    roi4 = cv2.cvtColor(roi3, cv2.COLOR_BGR2HSV)
    roi4 = draw_rectangle(roi4, lpos, rpos)

    # roi5 Upper Left Reigion for 4 signal lights
    roi5 = image[0:200, 180 : 310]
    roi5 = cv2.cvtColor(roi3, cv2.COLOR_BGR2HSV)
    roi5 = draw_rectangle(roi5, lpos, rpos)

    dist1 = cv2.inRange(roi2, lowerb1, upperb1)  # yellow(Below)
    dist2 = cv2.inRange(roi3, lowerb2, upperb2)  # red(Upper, 3 sigal lights)
    dist3 = cv2.inRange(roi3, lowerb3, upperb3)  # green(Upper, 3 signal lights)
    dist4 = cv2.inRange(roi4, lowerb2, upperb2)  # green(Upper, 4 signal lights)
    dist5 = cv2.inRange(roi5, lowerb3, upperb3)  # green(Upper, 4 signal lights)

    return dist1, dist2, dist3, dist4, dist5


# 빨간불을 인식하면 정지하고 초록불을 인식하면 출발한다.
def check_red_green(dist2, dist3):
    times_red = 0 
    times_green = 0 
    while True :
        for i in dist2:
            if 255 in i:
                times_red +=1
        for j in dist3:
            if 255 in j:
                times_green +=1
        if times_green >=10: 
            return Yellow = False

        if times_red >= 10:
            drive_out()
            return Yellow = True 


# 차 전방의 노란차선을 인식해 멈춘다.
def check_yellow(dist1): 
    yellow_cnt = 0
    times_yellow = 0
    for i in dist1:
        if 255 in i:
            times_yellow +=1
    if times_yellow >= 20:
        drive_out()
        Yellow = True 
    return Yellow

# 차 전방의 노란차선을 인식해 멈추고 일정 시간이 지나면 출발한다.
def check_yellow_(dist1): #without a green singal light detection
    yellow_cnt = 0
    times_yellow = 0
    for i in dist1:
        if 255 in i:
            times_yellow +=1
    if times_yellow >= 20:
        yellow_cnt += 1
        drive_out()
    if yellow_cnt >=5:
        Yellow = False
    return Yellow

# 마지막 신호등에서 왼쪽과 오른쪽 방향 중 어떤 신호등이 켜지는지 확인하고 그 방향으로 움직인다.
def check_green(dist4, dist5):
    times_left_green = 0
    times_right_green =0
    for n in dist5: 
        if 255 in n:
            times_right_green +=1

    for i in dist4: #Check Upper Right Region 
        if 255 in i:
            times_left_green += 1

    if times_right_green >= 5 or times_left_green >= 5:
        if tiems_right_green > times_left_green 
            Right_Green= True
            Left_Green = False 
        else: 
            Right_Green = False 
            Left_Green = True 

    if Right_Green == True:
        x = 0 
        while (X<8000):
            drive(20, 6)
        Right_Green = False 
        break 

    elif Left_Green == True:
        y = 0 
        while(y<7000):
            drive(-15, 6)
        Left_Green= False 
        break 


# 3갈래 길 미션에서 앞의 정지된 차를 인식한다.
def check_car(dist1): #fourth task: stop before the yellow line and detect the objects 
    ok = 0 #first detection
    ok2 = 0 #second detection after first one  
    while True:
        for degree in range(62, 117):
            if ranges_l[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_out()
                ok2 += 1 

        if ok2 >= 3:
            detect_car = True
            break 

    return detect_car 
            

# lidar로 전방의 장애물을 확인하며 움직인다.
def drive_lidar(type, angle):
    ok = 0  # cheking the obstacle
    drive(angle, 6)
    if type == 1: #normal drive
        for degree in range(62, 117):
            if ranges_l[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_out()
                break
    if type == 2:  #just only for second task
        for degree in range(62, 130):
            if ranges_l[degree] <= 0.3:
                ok += 1
            if ok > 3:
                drive_out()
                break

def first_task(type, angle, image, lpos, rpos:
    global dist2, dist3
    while True:
        drive_lidar(type, angle)
        _, dist2, dist3, _, _ = roi_process(image,lpos, rpos)
        Yellow = check_red_green(dist2, dist3)
        if Yellow == True:
            drive_out()

        if Yellow == False:
            drive_lidar(type, angle)
            task = change_state(2)
            break 

    return task


def second_task(type, angle, image, lpos, rpos):
    global dist2, dist3
    while True:
        drive_lidar(type, angle)
        _, dist2, dist3, _, _  = roi_process(image,lpos, rpos)
        Yellow = check_red_green(dist2, dist3)
        if Yellow == True:
            drive_out()

        if Yellow == False:
            drive_lidar(type, angle)
            task = change_state(3)
            break 

    return task


def thrid_task(type, angle, image, lpos, rpos):
    global dist2, dist3
    while True:
        drive_lidar(type, angle)
        _, dist2, dist3, _, _  = roi_process(image,lpos, rpos)
        Yellow  = check_red_green(dist2, dist3)
        
        if Yellow == True:
            drive_out()

        if Yellow == False:
            drive_lidar(type, angle)
            task = change_state(4)
            break 

    return task

def fourth_task(type, angle, image, lpos, rpos):
    global dist1, dist4, dist5
    a,b,c,d,e =0, 0, 0, 0, 0
    while True: 
        drive_lidar(type, angle)
        dist1, _, _, dist4, dist5 = roi_process(image, lpos, rpos): 
        Yellow = check_yellow_(dist1) #without a green signal light detection
        detect_car = check_car(dist1)
        if Yellow == True:
            drive_out()
        
        if Yellow == False and detect_car == True:
            #Backward
            while(a<8000):
                drive(0, -2)
                a+=1 
            drive_out()
            #Turn Left
            while(b<6500):
                drive(-40, 6)
                b+=1 
            #Turn Right
            while(c<8000):
                drive(40, 6)
                c+=1 
            #Turn Right
            while(d<7500):
                drive(37, 6)
                d+=1 
            #Turn Left
            while(e<3500):
                drive(-30, 6)
                e+=1
            drive_out() 
            ready = True #Ready for 4signal lights task 
        
        if ready == True: #final task 
            check_green(dist4, dist5)
            drive_lidar(type, angle)



def start():
    global image
    global cap
    global Width, Height
    global angle 
    global dist1, dist2, dist3 

    Yellow = False
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    laser_sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)
    task = 1
    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = process_image(image)
        dist1, dist2, dist3, dist4, dist5 = roi_process(image, lpos, rpos)
        center = (lpos + rpos) / 2
        angle = -(Width/2 - center)
        Yellow = check_yellow(dist1) #Yellow check loop 
        drive_lidar(1, angle) #normal drive 

        # first task: yellow line, obstacle1 
        if task == 1 and Yellow == True:
            drive_out()
            task = first_task(1, angle, image, lpos, rpos)
            Yellow = False

        # second task: yellow line, obstacle2
        if  task== 2 and Yellow == True:
            drive_out()
            task = second_task(2, angle, image, lpos, rpos)
            Yellow = False

        # third task: yellow_line 
        if task == 3 and Yellow == True:
            drive_out()
            task = third_task(1, angle, image, lpos, rpos)
            Yellow = False

        # firth task: yellow_line, Car 
        if task == 4 and Yellow == True:
            drive_out()
            fourth_task(1, angle, image, lpos, rpos)
            print("Complete")
            break 

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':
    start()


