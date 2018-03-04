#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import time
import serial
import struct
import numpy as np

import signal
import sys


START_BYTE = chr(251)
END_BYTE = chr(252)


MAX_MOTOR_VOLTAGE = 24.0
MAX_LED_VALUE = 255


class QuanserAero(object):

    def __init__(self, serial_port_path="/dev/ttyACM0"):
        self._serial_port_path = serial_port_path

        print('Making Serial Connection')
        self._serial_port = serial.Serial(self._serial_port_path, 115200, timeout=0, writeTimeout=0)
        print('Made Serial Connection')

        signal.signal(signal.SIGINT, self._signal_handler)
        print('Pressing Ctrl+C sends the zero signals to the motors')
     
        self._prev_time = time.time()
        self._prev_state = None

    def receive(self):
        message_length = 8
        message = bytearray(message_length)

        message_in_progress = False
        message_waiting = False
        message_index = 0

        while True and not message_waiting:
            part = self._serial_port.read()
            if message_in_progress:
                if message_index < message_length:
                    message[message_index] = part
                    message_index += 1
                elif part == END_BYTE:
                    message_in_progress = False
                    message_waiting = True
                    message_index = 0
            elif part == START_BYTE:
                message_in_progress = True

        pitch_part = message[0:4]
        yaw_part = message[4:8]

        pitch = struct.unpack('<f', pitch_part)[0]
        yaw = struct.unpack('<f', yaw_part)[0]

        return pitch, yaw


    def send(self, motor0_voltage, motor1_voltage, led_red, led_green, led_blue):

        motor0_voltage = max(-MAX_MOTOR_VOLTAGE, min(MAX_MOTOR_VOLTAGE, motor0_voltage))
        motor1_voltage = max(-MAX_MOTOR_VOLTAGE, min(MAX_MOTOR_VOLTAGE, motor1_voltage))

        led_red   = max(0, min(MAX_LED_VALUE, led_red))
        led_green = max(0, min(MAX_LED_VALUE, led_green))
        led_blue  = max(0, min(MAX_LED_VALUE, led_blue))

        message = bytearray(22)
        message[0] = START_BYTE

        # motor 0 voltage
        print("motor0: ", motor0_voltage)
        motor0_voltage_byte = bytearray(struct.pack("<f", motor0_voltage))
        message[1] = motor0_voltage_byte[0]
        message[2] = motor0_voltage_byte[1]
        message[3] = motor0_voltage_byte[2]
        message[4] = motor0_voltage_byte[3]

        # motor 1 voltage
        print("motor1: ", motor1_voltage)
        motor1_voltage_byte = bytearray(struct.pack("<f", motor1_voltage))
        message[5] = motor1_voltage_byte[0]
        message[6] = motor1_voltage_byte[1]
        message[7] = motor1_voltage_byte[2]
        message[8] = motor1_voltage_byte[3]

        # red led
        led_red_byte = bytearray(struct.pack("<i", led_red))
        message[9] = led_red_byte[0]
        message[10] = led_red_byte[1]

        # green led
        led_green_byte = bytearray(struct.pack("<i", led_green))
        message[11] = led_green_byte[0]
        message[12] = led_green_byte[1]

        # blue led
        led_blue_byte = bytearray(struct.pack("<i", led_blue))
        message[13] = led_blue_byte[0]
        message[14] = led_blue_byte[1]

        message[15] = END_BYTE
        self._serial_port.write(message)


    def step(self, action, to_print=False):
        motor0_voltage, motor1_voltage, led_red, led_green, led_blue = action
        send_time = time.time()

        self.send(motor0_voltage, motor1_voltage, led_red, led_green, led_blue)
        pitch, yaw = self.receive()

        if to_print == True:
            print(
                'pitch: {:.5f}'.format(pitch),
                'yaw: {:.5f}'.format(yaw),
                'motor_voltages (0,1): {:.5f}, {:.5f}'.format(motor0_voltage, motor1_voltage),
                'leds (R,G,B): {:.5f},{:.5f},{:.5f}'.format(led_red, led_green, led_blue),
                'time: ', send_time.__str__() )

        delta_time = time.time() - send_time

        # calculate deltas
        current_time = time.time()
        if self._prev_state == None: 
            d_pitch = 0
            d_yaw = 0
        else:
            d_pitch = (pitch - self._prev_state[0]) / (current_time - self._prev_time)
            d_yaw = (yaw - self._prev_state[1]) / (current_time - self._prev_time)

        self._prev_state = [pitch, yaw, d_pitch, d_yaw]
        self._prev_time = current_time

        return pitch, yaw, d_pitch, d_yaw


    def _signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        action = [0, 0,    255, 0, 0]
        self.step(action)
        print('Shutting down the Aero Safely')
        sys.exit(0)
