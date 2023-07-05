import torch
import numpy as np
import cv2
from robot import Robot
import time

min_thresh = 115
max_thresh = 255
robot = Robot()

def show_camera():
	if vcap.isOpened():
		#print(":)")
		while(1):
			ret, frame = vcap.read()
#			cv2.imshow('VIDEO', frame)
			cv2.waitKey(1)

			cropped = frame[200:600, 140:400]
			#cv2.imshow('Cropped', cropped)

			gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
			#cv2.imshow('Grayscale', gray)

			estThresh, threshold = cv2.threshold(gray, min_thresh, max_thresh, cv2.THRESH_BINARY)
			#cv2.imshow('Threshold',threshold)

			inverse = cv2.bitwise_not(threshold)
			#cv2.imshow('Inverse', inverse)

			contours, hierarchy = cv2.findContours(inverse, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
			if len(contours) > 0:
				contour = max(contours, key = len)
			else:
				contour = None

			contourImg = cv2.drawContours(cropped, contour, -1, (0, 255, 0), 3)
			#cv2.imshow('max contours', contourImg)

			frameCopy = np.copy(cropped)

			M = cv2.moments(contour)
			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])

				cv2.circle(contourImg, (cX, cY), 7, (255, 255, 255), -1)

#				cv2.imshow('Contours w/ centre', frame)
#				cv2.imshow('Cropped Contours', contourImg)
				if cX < 70:
					print("Turn Left")
					robot.left(1)
#					robot.right(-0.4)
					robot.stop()
					#time.sleep(0.4)
				elif cX > 210:
					print("Turn right")
					robot.right(1)
#					robot.left(-0.4)
					robot.stop()
					#time.sleep(0.4)
				else:
					print("Go forward")
					robot.forward(1)
					robot.stop()
					#time.sleep(0.4)
			else:
				print("Looking for line...")
				robot.right(1)
				robot.stop()

	else:
		print("Error: Unable to open camera")


if __name__ == "__main__":
	window_title = "CSI Camera"
	vcap = cv2.VideoCapture("rtsp://127.0.0.1:8554/test")
	show_camera()
