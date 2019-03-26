import cv2 as cv
import numpy as np
import serial
import time


COM_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
SERIAL_PORT = serial.Serial(COM_PORT, BAUD_RATE)
time.sleep(1)
print('Initialized serial port at', SERIAL_PORT.name)
total_pixels = 0
delay = 0.5


def send_data(text):
    print('Sending data to serial port:', text)
    SERIAL_PORT.write(text.encode('utf8'))
    SERIAL_PORT.flush(); # CHECK THIS


def calculate_ball_size(mask):
    return (cv.countNonZero(mask) / total_pixels) * 100


if __name__ == '__main__':
    cap = cv.VideoCapture(0)
    # Elliptic matrix as a kernel of detection
    morph_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
    total_pixels = cap.get(3) * cap.get(4)
    # Infinite loop - will be closed after clicking ESC
    while(True):
        # Take some amount of frames to fight with delays and save the last one
	for i in range (0, int(cap.get(cv.CAP_PROP_FPS)*delay)):
        	ret, frame = cap.read()
	if delay == 0:
		ret, frame = cap.read()
        # Convert RGB to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # First mask to define ranges of red color in HSV - more discriminating
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv.inRange(hsv, lower_red, upper_red)
        # Second mask - more tolerant
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([5, 255, 255])
        mask2 = cv.inRange(hsv, lower_red, upper_red)
        # Two masks combined
        mask_sum = mask1 + mask2
        # Bitwise AND of masks and original image - creates intersection of two images
        res1 = cv.bitwise_and(frame, frame, mask = mask1)
        res2 = cv.bitwise_and(frame, frame, mask = mask2)
        res_sum = cv.bitwise_and(frame, frame, mask = mask_sum)
        # Morphological transformation - erosion followed by dilation
        erosion = cv.erode(mask_sum, morph_kernel, iterations = 0) # 0 ITERATIONS - changed to avoid performance issues
        dilation = cv.dilate(erosion, morph_kernel, iterations = 0) # 0 ITERATIONS
	dilation = mask_sum
        # Get moment from opening
        moment = cv.moments(dilation)
        if(moment["m00"] != 0):
            # Calculate x and y coordinates of blob center from the moment
            cX = int(moment["m10"] / moment["m00"])
            cY = int(moment["m01"] / moment["m00"])
            # Convert cX to be readable by Arduino protocol, eg. 4 to 004, 20 to 020
	    strcX = str(cX)
            strcXOut = ''
	    for i in range(0, 3-len(strcX)):
	 	strcXOut += "0"
	    strcXOut += strcX
            # Add blue circle to the center of found blob to res_sum
            cv.circle(res_sum, (cX, cY), 5, (255, 0, 0), -1)
            # Sending found x coordinate of centroid to serial port
	    data_to_send = "r" + strcXOut
            send_data(data_to_send)
	    calculate_ball_size(dilation) # Not used yet
            time.sleep(delay)
        else:
            print("Could not calculate Cx, Cy - nothing is being send to Serial port")
	    data_to_send = "r700"
            send_data(data_to_send)
            time.sleep(delay)
        # Display windows
        # cv.imshow('camera image',frame)
        # cv.imshow('mask sum - used for calculations', mask_sum)
        # cv.imshow('mask1 res', res1)
        # cv.imshow('mask2 res', res2)
        # cv.imshow('mask_sum res', res_sum)
        # cv.imshow('mask_sum after morph erosion dilaion', dilation)
        # cv.imshow('mask_sum after morph erosion', erosion)
        # Program quits on ESC button click
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    cv.destroyAllWindows()
