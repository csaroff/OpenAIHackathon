from __future__ import division, with_statement, print_function

import numpy as np
import gym

from time import sleep
import keyboard #Using module keyboard

from collections import defaultdict
from collections import deque
import sys
import struct
import argparse
import time
from aero_bridge import QuanserAero

import csv
import logging
import datetime
from aero_bridge import MAX_MOTOR_VOLTAGE



class AeroEnv(gym.Env):
    def __init__(self, timeout=0.100, serial_port_path="/dev/ttyACM1", data_dir="data"):
        self._timeout = timeout
        self._data_dir = data_dir

        self._start_time = time.time()
        self._prev_state = None

        # self.action_space = gym.spaces.Box(low=np.array([-MAX_MOTOR_VOLTAGE, -MAX_MOTOR_VOLTAGE]), \
        #     high=np.array([MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE]))
        # self.observation_space = gym.space.Box(low=np.array([-100, -3000, -100, -100]), \
        #     high=np.array([100, 3000, 100, 100]))

        self._aero = QuanserAero(serial_port_path=serial_port_path)

        self._desired_pitch = 0
        self._desired_yaw = 0

    def reward_fn(self, prev_state, action, state):
        return -(state[0]-self._desired_pitch)**2 - (state[1]-self._desired_yaw)**2 - state[2]**2 - state[3]**2


    def reset(self):
        state = self._aero.send(0, 0, 0, 255, 0)
        self._start_time = time.time()
        return state, 0, False, {}

    def done(self, state):
        if abs(self._desired_pitch-state[0]) < 10 and abs(self._desired_yaw-state[1]) < 10: 
            return True
        else: 
            return False

    def step(self, action):
        # ****** wait for timeout to get consistent timing ******
        waited = False
        loop_start_time = time.time()
        while time.time() - self._start_time <= self._timeout:
            waited = True
            sleep(0.001)

        if waited == False: 
            print("WARNING: Time outside loop was too great")
        else: 
            print("Timing was good with extra time: %s" % (self._timeout - (loop_start_time-self._start_time)) )

        self._start_time = time.time()
        # ****** end of timing code ******

        # get state
        state = self._aero.step(action)
        reward = self.reward_fn(self._prev_state, action, state)

        done = self.done(state)

        self._prev_state = state
        return state, reward, done, {}



