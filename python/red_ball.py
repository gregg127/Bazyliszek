import cv2 as cv
import numpy as np
import serial
import time
import sys

COM_PORT = '/dev/ttyUSB0' # for debug: COM1
BAUD_RATE = 9600
DELAY = 0.5
EROSION_ITERATIONS = 0
DILATION_ITERATIONS = 0


def init():
    if len(sys.argv) == 2 and sys.argv[1] == "help":
        print("Program is executed with 2 arguments.")
        print("First argument is mode in which program runs:")
        print("\tdebug - for desktop color detection debugging")
        print("\tserial - for robot deployment - sends data to serial port and does not display captured frames")
        print("Second argument is the color to detect:")
        print("\tred")
        print("\tgreen")
        print("\tblue")
        print("Example program executions:")
        print("\tscript_name.py debug red")
        print("\tscript_name.py serial green")
        sys.exit()
    if len(sys.argv) != 3:
        print("Invalid number of arguments. Expected args: mode color, for example")
        print("red_ball.py debug green")
        print("red_ball.py serial red")
        print("Run script with argument 'help' to get more info")
        sys.exit()
    elif sys.argv[1] not in ["serial", "debug"]:
        print("Invalid first argument, expected values: serial, debug")
        print("Run script with argument 'help' to get more info")
        sys.exit()
    elif sys.argv[2] not in ["red", "green", "blue"] :
        print("Invalid second argument, expected values: red, green, blue")
        print("Run script with argument 'help' to get more info")
        sys.exit()
    else:
        print("Starting program...")
        global SERIAL_PORT
        global total_pixels
        global COLOR
        global MODE
        MODE = sys.argv[1]
        COLOR = sys.argv[2]
        if MODE == "serial":
            print("Initializing serial port...")
            SERIAL_PORT = serial.Serial(COM_PORT, BAUD_RATE)
            time.sleep(1)
            print("Serial port initialized at " + SERIAL_PORT.name)
        total_pixels = 0


def get_first_mask_colors():
    if COLOR == "green":
        return 0
    if COLOR == "red":
        return np.array([170, 50, 50]), np.array([180, 255, 255]) 
    if COLOR == "blue":
        return 0
    return -1


def get_second_mask_colors():
    if COLOR == "green":
        return 0
    if COLOR == "red":
        return np.array([0, 50, 50]), np.array([5, 255, 255])
    if COLOR == "blue":
        return 0
    return -1


def get_protocol_flag(cX):
    # Convert cX to be readable by Arduino protocol, eg. 4 to 004, 20 to 020
    strcX = str(cX)
    strcXOut = ''
    for i in range(0, 3-len(strcX)):
        strcXOut += "0"
    strcXOut += strcX
    return "r" + strcXOut


def send_data(text):
    print('Sending data to serial port:', text)
    SERIAL_PORT.write(text.encode('utf8'))
    SERIAL_PORT.flush(); # TODO: check this


def calculate_ball_size(mask):
    return (cv.countNonZero(mask) / total_pixels) * 100


if __name__ == '__main__':
    init()
    cap = cv.VideoCapture(0)
    # Elliptic matrix as a kernel of detection
    morph_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    total_pixels = cap.get(3) * cap.get(4)
    lower_color_first, upper_color_first = get_first_mask_colors()
    lower_color_second, upper_color_second = get_second_mask_colors()
    while(True):
        # Take some amount of frames to fight with delays and save the last one
        for i in range(0, int(cap.get(cv.CAP_PROP_FPS) * DELAY)):
        	ret, frame = cap.read()
        if DELAY == 0:
            ret, frame = cap.read()
        # Convert RGB to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # First mask to define ranges of color in HSV - more discriminating
        mask1 = cv.inRange(hsv, lower_color_first, upper_color_first)
        # Second mask - more tolerant
        mask2 = cv.inRange(hsv, lower_color_second, upper_color_second)
        # Two masks combined
        mask_sum = mask1 + mask2
        # Bitwise AND of masks and original image - creates intersection of two images
        res1 = cv.bitwise_and(frame, frame, mask = mask1)
        res2 = cv.bitwise_and(frame, frame, mask = mask2)
        res_sum = cv.bitwise_and(frame, frame, mask = mask_sum)
        # Morphological transformation - erosion followed by dilation
        erosion = cv.erode(mask_sum, morph_kernel, iterations = EROSION_ITERATIONS)
        dilation = cv.dilate(erosion, morph_kernel, iterations = DILATION_ITERATIONS)
        # Get moment from opening
        moment = cv.moments(dilation)
        if(moment["m00"] != 0):
            # Calculate x and y coordinates of blob center from the moment
            cX = int(moment["m10"] / moment["m00"])
            cY = int(moment["m01"] / moment["m00"])
            # Add blue circle to the center of found blob to res_sum
            cv.circle(res_sum, (cX, cY), 5, (255, 0, 0), -1)
            # Sending found x coordinate of centroid to serial port
            if MODE == "serial":
                data_to_send = get_protocol_flag(cX)
                send_data(data_to_send)
                time.sleep(DELAY)
            calculate_ball_size(dilation) # Not used yet
        else:
            print("Could not calculate Cx, Cy")
            if MODE == "serial":
                send_data("r700") # TODO: HARDCODED - change that
                time.sleep(DELAY)
        if MODE == "debug":
            # cv.imshow('camera image',frame)
            # cv.imshow('mask sum - used for calculations', mask_sum)
            # cv.imshow('mask1 res', res1)
            # cv.imshow('mask2 res', res2)
            cv.imshow('mask_sum res', res_sum)
            cv.imshow('mask_sum after morph erosion and dilation', dilation)
            # cv.imshow('mask_sum after morph erosion', erosion)
            # Program quits on ESC button click
            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break
    cv.destroyAllWindows()
