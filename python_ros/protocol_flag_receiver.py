#!/usr/bin/env python

from signal import signal, SIGINT
from sys import exit
import rospy
import os
import time
import serial
from multiple_machines.msg import ProtocolMessage

# Otwarcie portu szeregowego
arduino = serial.Serial('/dev/ttyUSB0', 250000, timeout=.3)

def callback(data):
	# Zmienna zawierajaca tresc otrzymanej wiadomosci
	info = format_message(data)
	# Wyswietlenie informacji o otrzymanej wiadomosci
	rospy.loginfo('Received message ' + info)
	# Wyslanie wiadomosci po porcie szeregowym do Arduino
	arduino.write(info)

def listener():
	# Zerejestrowanie wezla
	rospy.init_node('protocol_message_subscriber')
	# Zasubskrybowanie tematu
	rospy.Subscriber('protocol_sender', ProtocolMessage, callback)
	rospy.spin()

def sigint_handler(signal_received, frame):
	print('Stopping robot and quitting program...')
	arduino.write('v000')
	exit(0)

def format_message(message):
    return str(message.flag) + str(message.value).zfill(3)

if __name__ == '__main__':
	print('Program started...')
	# Dodanie obslugi sygnalu przerwania programu
	signal(SIGINT, sigint_handler)
	listener()
