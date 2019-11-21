#!/usr/bin/env python

import rospy
from multiple_machines.msg import ProtocolMessage

# Zmienne globalne
ROSPY_RATE = 20
PROTOCOL_MESSAGE_LENGTH = 4
PROTOCOL_FLAGS = {
    "MOVE_FORWARD": "m",
    "MOVE_BACKWARD": "b",
    "ROTATE_RIGHT": "r",
    "ROTATE_LEFT": "l",
    "SET_VELOCITY": "v"
}

def talker():
    global pub
    global rate
    # Zdefiniowanie publishera ktory bedzie przyjmowal informacje
    pub = rospy.Publisher('chatter', ProtocolMessage, queue_size=10) # publikuje pod nazwa chatter
    # Inicjalizacja wezla wysylajacego informacje
    rospy.init_node('talker', anonymous=True)
    # Ustalenie maksymalnej czestotliwosci wysylania informacji
    rate = rospy.Rate(ROSPY_RATE) # 20hz
    # Glowna petla wezla
    run_node(rospy, pub, rate)
    
def run_node(rospy, pub, rate):
    while not rospy.is_shutdown():
        # Wczytanie stringa z konsoli
        data = raw_input("Enter protocol flag (eg. v250): ")
        
        if len(data) == PROTOCOL_MESSAGE_LENGTH and is_flag_valid(data) and is_value_valid(data):
            protocol_message = ProtocolMessage()
            protocol_message.flag = get_protocol_flag(data)
            protocol_message.value = get_protocol_value(data)
            serve_protocol(protocol_message)
        else:
            print("Entered invalid protocol packet. Try again.")
            print("Available flags: ")
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
        print("Exception occured during flag validation")
        return False 

def is_value_valid(data):
    try:
        value = int(get_protocol_value(data))
        return 0 <= value <= 999
    except:
        print("Exception occured during value validation") 
        return False

def serve_protocol(protocol_message):
    flag = protocol_message.flag
    if flag == PROTOCOL_FLAGS["MOVE_FORWARD"]:
        serve_move_forward(protocol_message)
    elif flag == PROTOCOL_FLAGS["MOVE_BACKWARD"]:
        serve_move_backward(protocol_message)
    elif flag == PROTOCOL_FLAGS["ROTATE_RIGHT"]:
        serve_rotate_right(protocol_message)
    elif flag == PROTOCOL_FLAGS["ROTATE_LEFT"]:
        serve_rotate_left(protocol_message)
    elif flag == PROTOCOL_FLAGS["SET_VELOCITY"]:
        serve_set_velocity(protocol_message)

def serve_move_forward(protocol_message):
    print("\nMove backward")
    protocol_message.value = parse_value(protocol_message.value, 0, 999)
    ros_publish_message(protocol_message)

def serve_move_backward(protocol_message):
    print("\nMove forward")
    protocol_message.value = parse_value(protocol_message.value, 0, 999)
    ros_publish_message(protocol_message)

def serve_rotate_right(protocol_message):
    print("\nRotate right")
    protocol_message.value = parse_value(protocol_message.value, 0, 360)
    ros_publish_message(protocol_message)

def serve_rotate_left(protocol_message):
    print("\nRotate left")
    protocol_message.value = parse_value(protocol_message.value, 0, 360)
    ros_publish_message(protocol_message)

def serve_set_velocity(protocol_message):
    print("\nVelocity")
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
