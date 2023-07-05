import numpy as np
import cv2
from robot import Robot
import time

#Set global variables
center = 200
right_thresh = 230
left_thresh = 170
robot = Robot()
robot_x = 0
robot_y = 0
robot_heading = 0

kp_distance = 0.1
kp_heading = 0.1

#Function to automatically adjust the canny edge detection parameters
def auto_canny_edge_detection(image, sigma):
	md = np.median(image)
	lower_value = int(max(0, (1.0-sigma) * md))
	upper_value = int(min(255, (1.0+sigma) * md))
	return cv2.Canny(image, lower_value, upper_value)


def show_camera():
	window_title = "CSI Camera"
	vcap = cv2.VideoCapture("rtsp://127.0.0.1:8554/test")
	if vcap.isOpened():
		while(1):
			ret, frame = vcap.read()
			cv2.imshow('VIDEO', frame)
			cv2.waitKey(1)	

			#Change the colour to grayscale
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			#Invert the colours
			img_not = cv2.bitwise_not(gray)

			kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
			# Apply the sharpening kernel to the image using filter2D
			sharpened = cv2.filter2D(img_not, -1, kernel)

			# Apply a Gaussian blur to the image
			blurred = cv2.GaussianBlur(sharpened, (3, 3), 0)

			#Canny edge detection
			canny = auto_canny_edge_detection(blurred, 0.5)

			rho = 1  # distance resolution in pixels of the Hough grid
			theta = np.pi / 180  # angular resolution in radians of the Hough grid
			threshold = 15  # minimum number of votes (intersections in Hough grid cell)
			min_line_length = 200  # minimum number of pixels making up a line
			max_line_gap = 30  # maximum gap in pixels between connectable line segments
			line_image = np.copy(frame) * 0  # creating a blank to draw lines on

			# Run Hough on edge detected image
			# Output "lines" is an array containing line segments
			lines = cv2.HoughLinesP(canny, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
			if lines is not None:
				for line in lines:
					for x1,y1,x2,y2 in line:
						cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
						line_location = ((x1 + x2) / 2, (y1 + y2) / 2)
						line_orientation = np.arctan2(y2 - y1, x2 - x1)

						#Finding the position of the line in the frame
						distance_error = line_location[0] - robot_x
						heading_error = line_orientation - robot_heading

						left_wheel_speed = kp_distance * distance_error - kp_heading * heading_error
						right_wheel_speed = kp_distance * distance_error + kp_heading * heading_error

						#Moving the robot so that the line is in the center of the frame
						if line_location[0] < left_thresh:
							print("turn left")
							robot.left(0.4)
							time.sleep(0.1)
						elif line_location[0] > right_thresh:
							print("Turn right")
							robot.right(0.4)
							time.sleep(0.1)
						else:
							print("forward")
							robot.forward(0.4)
							time.sleep(0.1)

						# Draw the lines on the image and display it
						lines_edges = cv2.addWeighted(frame, 1, line_image, 1, 0)
						cv2.imshow("lines_edges", lines_edges)

	else:
		print("Error: Unable to open camera")


if __name__ == "__main__":
	show_camera()
