#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, rospkg, time
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import sys
import os
import signal

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 150
ack_msg = xycar_motor()
ack_publisher = None
ranges_l = [] 
times_yellow=0 
times_red=0  
times_green=0 
part=0
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

def drive_out():
    global ack_msg, ack_publisher
    for stop_cnt in range(10):
        ack_msg.speed = 0
        ack_msg.angle = 0 
        ack_publisher.publish(ack_msg)
        time.sleep(0.1) # Stop

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
#색 검정색으로 change
        color = (0, 0, 0)
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

#색 검정색으로 change
# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 0, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 0, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 0, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 0), 2)
    return img

# left lines, right lines
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


def start():
    global Offset, Gap
    global times_yellow
    global times_red
    global times_green
    global pub
    global image
    global cap
    global Width, Height
    global ack_publisher
    global part


 
    rospy.init_node('auto_drive')
    ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    pub = ack_publisher
    rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)


    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)
    if part == 0:
        Yellow = False
	yellow_cnt = 0
	ok2 = 0
	times_yellow = 0
        while not rospy.is_shutdown():
            while not image.size == (640*480*3):
                continue


	    if yellow_cnt >=5:
#	  	print('yellow_go:', yellow_cnt)
            	drive(angle, 6) #angle->0
	    	ok = 0
	    	for degree in range(62,117):
	            if ranges_l[degree] <= 0.3:
		        ok += 1
	       	    if ok > 3:
                        drive_out()
			break
			
		if times_yellow == 0:
		    yellow_cnt = 0
		 	
	    times_yellow = 0

   	    # gray
   	    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

   	    # blur
   	    kernel_size = 5
   	    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    	    # canny edge
    	    low_threshold = 60
    	    high_threshold = 70
    	    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    	    # HoughLinesP
    	    roi = edge_img[Offset : Offset+Gap+20, 0 : Width]
    	    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    	    #roi2 Under Reigion
    	    roi2=image[Offset : Offset+Gap+20, 0 : Width]

    	    # divide left, right lines
    	    if all_lines is None:
        	    return 0, 640
    	    left_lines, right_lines = divide_left_right(all_lines) #

    	    # get center of lines
            frame, lpos = get_line_pos(image, left_lines, left=True)
            frame, rpos = get_line_pos(frame, right_lines, right=True)

	    # center, angle
            center = (lpos + rpos) / 2
            angle = -(Width/2 - center)

            # draw lines
            frame = draw_lines(frame, left_lines)
            frame = draw_lines(frame, right_lines)
            frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

            # draw rectangle
            frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    	    #roi2 Under Reigion
            roi2 = cv2.cvtColor(roi2, cv2.COLOR_BGR2HSV)
            roi2 = draw_rectangle(roi2, lpos, rpos)

            #hsv yellow
            lowerb1 = (20,100,100)
            upperb1 = (40,150,150)


            dist1=cv2.inRange(roi2, lowerb1, upperb1) #yellow

 	    # Recognizing Yellow Line
            for j in dist1:  
	            if 255 in j:
	                times_yellow +=1
#	    print('times_yellow',times_yellow)

            if times_yellow >= 20:
	            yellow_cnt += 1
#	            print('yellow_cnt', yellow_cnt)
#	            print('Stop Yellow line')
		    Yellow = True
		    if ok2 >= 3:
                        Yellow = False

            else:
                Yellow = False

	    if yellow_cnt >= 5:
		Yellow = False

	    if Yellow == False:
		    drive(angle, 6)

		    ok = 0
	    	    for degree in range(62,130):
			    if ranges_l[degree] <= 0.3:
		  		    ok += 1				
	       	 	    if ok > 3:
				    drive_out()
			    	    ok2 += 1	
				    break
#		    print("ok2:",ok2)
		    if ok2 >= 3:
			a=0
			b=0
        		c=0
        		d=0
        		e=0
			while(a<8000):
				drive(0, -2)
				a+=1
			drive_out()
	        	while(b<6500):
				drive(-40, 6)
				b+=1
#			drive_out()

			while(c<8000):
				drive(40, 6)
				c+=1
#			drive_out()

			while(d<7500):
				drive(37, 6)
				d+=1
#			drive_out()

			while(e<3500): #수정할 것
				drive(-30, 6)
				e+=1
			drive_out()
			part = 1
			break			
					 
	    else:
		    drive_out()
    
    if part == 1:
	print('Welcome to Part1')
	Right_Green = False
	Left_Green = False
        Yellow = False
        while not rospy.is_shutdown():
            while not image.size == (640*480*3):
                continue

    	    times_yellow = 0
	    times_right_green=0
	    times_left_green=0

   	    # gray
   	    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

   	    # blur
   	    kernel_size = 5
   	    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    	    # canny edge
    	    low_threshold = 60
    	    high_threshold = 70
    	    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    	    # HoughLinesP
    	    roi = edge_img[Offset : Offset+Gap+20, 0 : Width]
    	    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    	    #roi2 Under Reigion
    	    roi2=image[Offset : Offset+Gap+20, 0 : Width]

            #roi3 UPPER RIGHT REGION, Checking right .side green light
	    roi3=image[0:200, 330: 500]

            #roi4 UPPER LEFT REGION, Checking left side green light
	    roi4=image[0:200, 180: 310]

    	    # divide left, right lines
    	    if all_lines is None:
        	    return 0, 640
    	    left_lines, right_lines = divide_left_right(all_lines) #

    	    # get center of lines
            frame, lpos = get_line_pos(image, left_lines, left=True)
            frame, rpos = get_line_pos(frame, right_lines, right=True)

	    # center, angle
            center = (lpos + rpos) / 2
            angle = -(Width/2 - center)

            # draw lines
            frame = draw_lines(frame, left_lines)
            frame = draw_lines(frame, right_lines)
            frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

            # draw rectangle
            frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    	    #roi2 Under Reigion
            roi2 = cv2.cvtColor(roi2, cv2.COLOR_BGR2HSV)
            roi2 = draw_rectangle(roi2, lpos, rpos)

	    #roi3 UPPER RIGHT REGION, Checking right side green light
            roi3 = cv2.cvtColor(roi3, cv2.COLOR_BGR2HSV)

            #roi4 UPPER LEFT REGION, Checking left side green light
            roi4 = cv2.cvtColor(roi4, cv2.COLOR_BGR2HSV)

            #hsv yellow
            lowerb1 = (20,100,100)
            upperb1 = (40,150,150)

            #hsv green
            lowerb3 = (60,70,70)
            upperb3 = (80,150,150)

            dist1=cv2.inRange(roi2, lowerb1, upperb1) #yellow
            dist3=cv2.inRange(roi3, lowerb3, upperb3) #green, Upper, Right 
            dist4=cv2.inRange(roi4, lowerb3, upperb3) #green, Upper, Left

            for j in dist1:   
	    	if 255 in j:
	        	times_yellow +=1
#	    print('times_yellow',times_yellow)


            if times_yellow >= 20:
#	    	print('Stop Yellow line')
            	Yellow = True
            else:
            	Yellow = False

	    if Yellow == False:
		drive(angle, 6)

		ok = 0
	    	for degree in range(62,117):
			if ranges_l[degree] <= 0.3:
		  		ok += 1
	       	 	if ok > 3:
				drive_out()
				break
				

            elif Yellow == True:
    	    	for r in dist3:
  	            if 255 in r:
		    	times_right_green +=1
#	    	print('times_right_green',times_right_green)

		for l in dist4:
  	            if 255 in l:
		    	times_left_green +=1
#	    	print('times_left_green',times_left_green)

	    	if times_right_green >= 5 or times_left_green >= 5:
		     if times_right_green > times_left_green:
			  Right_Green = True
			  Left_Green = False
		     else:
			  Left_Green = True
			  Right_Green = False

        # if detect right_green, then while(3000) go forward and turn right  
	        if Right_Green==True:
                     x=0
	             x2=0
		     print('Go right')
                     while(x<8000):   
                         drive(20, 6)  #need to check  
		         x += 1
          
		     Right_Green = False
		     print('Right Complete')
		     part = 2
		     break

        # if detect left_green, then while(3000) go forward and turn left  
	        elif Left_Green==True:
                     y = 0
		     print('Go left')
                     while(y<7000):
                         drive(-15, 6)  #need to check
		         y += 1   
             
                     Left_Green = False
		     print('Left Complete')
		     part = 2
		     break   
          
	        else:
		     drive_out()
	    else:
		print('Error2')
       

    if part == 2:
	print('Welcome to Part2')
	
        while not rospy.is_shutdown():
	    
            while not image.size == (640*480*3):
                continue

   	    # gray
   	    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

   	    # blur
   	    kernel_size = 5
   	    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    	    # canny edge
    	    low_threshold = 60
    	    high_threshold = 70
    	    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    	    # HoughLinesP
    	    roi = edge_img[Offset : Offset+Gap+20, 0 : Width]
    	    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    	    # divide left, right lines
    	    if all_lines is None:
        	    return 0, 640
    	    left_lines, right_lines = divide_left_right(all_lines) #

    	    # get center of lines
            frame, lpos = get_line_pos(image, left_lines, left=True)
            frame, rpos = get_line_pos(frame, right_lines, right=True)

	    # center, angle
            center = (lpos + rpos) / 2
            angle = -(Width/2 - center)

            # draw lines
            frame = draw_lines(frame, left_lines)
            frame = draw_lines(frame, right_lines)
            frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

            # draw rectangle
            frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
	    drive(angle, 6)
	    ok = 0
	    for degree in range(62,117):
			if ranges_l[degree] <= 0.35:
		  		ok += 1
	       	 	if ok > 3:
				drive_out()
				break

        rospy.spin()

if __name__ == '__main__':

    start()


