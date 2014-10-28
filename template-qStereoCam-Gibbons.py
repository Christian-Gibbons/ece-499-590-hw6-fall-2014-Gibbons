#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import operator
from math import sin

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

center_screen = (nx/2,ny/2)
hue = 60
h_thresh = 10
lower_hue = np.zeros((ny, nx, 3), np.uint8)
upper_hue = np.zeros((ny, nx, 3), np.uint8)
lower_hue[:] = (hue-h_thresh,50,50)
upper_hue[:] = (hue+h_thresh,255,255)

FOV = 1.047
f = 0.085
b = 0.4
pixel_size = 0.000280

trackedL = False
trackedR = False
errorL = (0,0)
errorR = (0,0)

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
	# Get Frame
	imgL = np.zeros((newx,newy,3), np.uint8)
	imgR = np.zeros((newx,newy,3), np.uint8)
	c_image = imgL.copy()
	c_image = imgR.copy()
	vidL = cv2.resize(c_image,(newx,newy))
	vidR = cv2.resize(c_image,(newx,newy))
	[status, framesize] = vl.get(vidL, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		vid2 = cv2.resize(vidL,(nx,ny))
		imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)

		imgL2 = cv2.cvtColor(imgL,cv2.COLOR_RGB2HSV)
		maskL = cv2.inRange(imgL2, lower_hue, upper_hue)
		mL = cv2.moments(maskL)

		#if the object is not on screen, hold on to it's last known location if there is one
		if(mL['m00'] == 0.0):
			if(trackedL):
#				print 'last known location of center: ', object_center
				print 'last known error: ', errorL

		#if it is on screen, find it's location and how far it is from center
		else:
			trackedL = True
			object_centerL = (int (mL['m10']/mL['m00']), int (mL['m01']/mL['m00']))
			errorL = tuple(map(operator.sub, object_centerL, center_screen))

#			print 'current location of center: ', object_center
#			print 'current error: ', error

#			print errorL

#			(cvPIDL.X,cvPIDL.Y) = object_center
#			cvt.put(cvPIDL)
#			print '(', cvPID.X, ',', cvPID.Y, '), (', cvPID.setX, ',', cvPID.setY, ')' 

		#draw red dot in center of the (visible part of the) object if it's still on screen
		if(trackedL):
			if(mL['m00'] != 0.0):
				resL = cv2.circle(imgL, object_centerL, 3, (0,0,255))
		cv2.imshow("wctrl_L", imgL)
		cv2.waitKey(10)
	else:
		raise ach.AchException( v.result_string(status) )
	[status, framesize] = vr.get(vidR, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		vid2 = cv2.resize(vidR,(nx,ny))
		imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)

		imgR2 = cv2.cvtColor(imgR,cv2.COLOR_RGB2HSV)
		maskR = cv2.inRange(imgR2, lower_hue, upper_hue)
		mR = cv2.moments(maskR)

		#if the object is not on screen, hold on to it's last known location if there is one
		if(mR['m00'] == 0.0):
			if(trackedR):
#				print 'last known location of center: ', object_center
				print 'last known error: ', errorR

		#if it is on screen, find it's location and how far it is from center
		else:
			trackedR = True
			object_centerR = (int (mR['m10']/mR['m00']), int (mR['m01']/mR['m00']))
			errorR = tuple(map(operator.sub, object_centerR, center_screen))

#			print 'current location of center: ', object_center
#			print 'current error: ', error

#			print errorR

#			(cvPIDR.X,cvPIDR.Y) = object_centerR
#			cvt.put(cvPID)
#			print '(', cvPID.X, ',', cvPID.Y, '), (', cvPID.setX, ',', cvPID.setY, ')' 

		#draw red dot in center of the (visible part of the) object if it's still on screen
		if(trackedR):
			if(mR['m00'] != 0.0):
				resR = cv2.circle(imgR, object_centerR, 3, (0,0,255))
		cv2.imshow("wctrl_R", imgR)
		cv2.waitKey(10)
	else:
		raise ach.AchException( v.result_string(status) )


	[status, framesize] = t.get(tim, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		pass
		#print 'Sim Time = ', tim.sim[0]
	else:
		raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
	# Def:
	# ref.ref[0] = Right Wheel Velos
	# ref.ref[1] = Left Wheel Velos
	# tim.sim[0] = Sim Time
	# imgL       = cv image in BGR format (Left Camera)
	# imgR       = cv image in BGR format (Right Camera)


	# Sleeps
	if(trackedL and trackedR):
		print errorL, errorR
		errXL, errYL = errorL
		errXR, errYR = errorR
		centerX, centerY = center_screen
		thetaL = (float(errXL-centerX)/float(newx)) * 1.047
		thetaR = (float(errXR-centerX)/float(newx)) * 1.047
		uL = f*sin(thetaL)/sin(3.14 + thetaL)
		uR = f*sin(thetaR)/sin(3.14 + thetaR)
		disparity = (uL - uR)/pixel_size
		depth = f*b/disparity
		print "depth: ", depth
	time.sleep(0.1)   
#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
