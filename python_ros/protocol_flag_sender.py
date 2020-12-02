#!/usr/bin/env python

import rospy
import sys, termios, tty, os, time
from multiple_machines.msg import ProtocolMessage

ROSPY_RATE = 20
PROTOCOL_MESSAGE_LENGTH = 4
PROTOCOL_FLAD_SEND_DELAY = 0.5
ROBOT_VELOCITY_INTERVAL = 10
PROTOCOL_FLAGS = {
    'MOVE_FORWARD': 'm',
    'MOVE_BACKWARD': 'b', 
    'ROTATE_RIGHT': 'r', 
    'ROTATE_LEFT': 'l', 
    'SET_VELOCITY': 'v' 
}

def talker():
    global pub
    global rate

    pub = rospy.Publisher('protocol_sender', ProtocolMessage, queue_size=10)
    rospy.init_node('protocol_message_sender')
    rate = rospy.Rate(ROSPY_RATE)

    run_node(rospy, pub, rate)

def run_node(rospy, pub, rate):
    current_velocity = 0
    robot_move = 50
    robot_rotate = 30

    while not rospy.is_shutdown():
        print('\nWaiting for input...')
        char = getch()
        if char == 'u':
            robot_move += 10
            robot_move = parse_value(robot_move, 0, 999)
            print('Updated distance: ' + str(robot_move))
        elif char == 'j':
            robot_move -= 10
            robot_move = parse_value(robot_move, 0, 999)
            print('Updated distance: ' + str(robot_move))
        elif char == 'o':
            robot_rotate += 10
            robot_rotate = parse_value(robot_rotate, 0, 360)
            print('Updated rotation angle: ' + str(robot_rotate))
        elif char == 'l':
            robot_rotate -= 10
            robot_rotate = parse_value(robot_rotate, 0, 360)
            print('Updated rotation angle: ' + str(robot_rotate))
        elif char == 'i':
            current_velocity += ROBOT_VELOCITY_INTERVAL
            current_velocity = parse_value(current_velocity, 0, 255)
            serve_user_input(PROTOCOL_FLAGS['SET_VELOCITY'] + format_value(current_velocity))
            wait()
        elif char == 'k':
            current_velocity -= ROBOT_VELOCITY_INTERVAL
            current_velocity = parse_value(current_velocity, 0, 255)
            serve_user_input(PROTOCOL_FLAGS['SET_VELOCITY'] + format_value(current_velocity))
            wait()
        elif char == 'w':
            serve_user_input(PROTOCOL_FLAGS['MOVE_FORWARD'] + format_value(robot_move))
            wait()
        elif char == 's':
            counter = 100
            serve_user_input(PROTOCOL_FLAGS['MOVE_BACKWARD'] + format_value(robot_move))
            wait()
        elif char == 'a':
            serve_user_input(PROTOCOL_FLAGS['ROTATE_RIGHT'] + format_value(robot_rotate))
            wait()
        elif char == 'd':
            serve_user_input(PROTOCOL_FLAGS['ROTATE_LEFT'] + format_value(robot_rotate))
            wait()
        elif char == 'q':
            print('Exitting...')
            exit()
        else:
            print('Character not found.')

def serve_user_input(data):
    if len(data) == PROTOCOL_MESSAGE_LENGTH and is_flag_valid(data) and is_value_valid(data):
        protocol_message = ProtocolMessage()
        protocol_message.flag = get_protocol_flag(data)
        protocol_message.value = get_protocol_value(data)
        serve_protocol(protocol_message)
    else:
        print('Entered invalid protocol packet. Try again.')
        print('Available flags: ')
        for key in PROTOCOL_FLAGS:
            print('\t {} -> {}'.format(key, PROTOCOL_FLAGS[key]))

def get_protocol_flag(packet):
    return packet[:1]

def get_protocol_value(packet):
    return packet[1:]

def is_flag_valid(data):
    try:
        flag = get_protocol_flag(data)
        return flag in PROTOCOL_FLAGS.values()
    except:
        print('Exception occured during flag validation')
        return False

def is_value_valid(data):
    try:
        value = int(get_protocol_value(data))
        return 0 <= value <= 999
    except:
        print('Exception occured during value validation')
        return False

#########################################
#           FUNKCJE STERUJACE
#########################################
def serve_protocol(protocol_message):
    flag = protocol_message.flag
    if flag == PROTOCOL_FLAGS['MOVE_FORWARD']:
        serve_move_forward(protocol_message)
    elif flag == PROTOCOL_FLAGS['MOVE_BACKWARD']:
        serve_move_backward(protocol_message)
    elif flag == PROTOCOL_FLAGS['ROTATE_RIGHT']:
        serve_rotate_right(protocol_message)
    elif flag == PROTOCOL_FLAGS['ROTATE_LEFT']:
        serve_rotate_left(protocol_message)
    elif flag == PROTOCOL_FLAGS['SET_VELOCITY']:
        serve_set_velocity(protocol_message)

def serve_move_forward(protocol_message):
    print('Move forward')
    protocol_message.value = parse_value(protocol_message.value, 0, 999)
    ros_publish_message(protocol_message)

def serve_move_backward(protocol_message):
    print('Move backward')
    protocol_message.value = parse_value(protocol_message.value, 0, 999)
    ros_publish_message(protocol_message)

def serve_rotate_right(protocol_message):
    print('Rotate right')
    protocol_message.value = parse_value(protocol_message.value, 0, 360)
    ros_publish_message(protocol_message)

def serve_rotate_left(protocol_message):
    print('Rotate left')
    protocol_message.value = parse_value(protocol_message.value, 0, 360)
    ros_publish_message(protocol_message)

def serve_set_velocity(protocol_message):
    print('Velocity')
    protocol_message.value = parse_value(protocol_message.value, 0, 255)
    ros_publish_message(protocol_message)

def parse_value(value, min, max):
    value = int(value)
    value = max if value > max else value
    value = min if value < min else value
    return value

def ros_publish_message(protocol_message):
    rospy.loginfo(protocol_message)
    pub.publish(protocol_message)
    rate.sleep()

#########################################
#           FUNKCJE POMOCNICZE
#########################################
def format_value(value):
    return str(value).zfill(3)

def wait():
    print('Sending data...')
    time.sleep(0.5)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#########################################
#           WYWOLANIE PROGRAMU
#########################################
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
