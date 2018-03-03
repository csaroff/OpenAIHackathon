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


def reward_fn(prev_state, action, state):
    return 0

class AeroEnv(gym.Env):

    def __init__(self, timeout=0.100, serial_port_path="/dev/ttyACM1", data_dir="data"):
        self._timeout = timeout
        self._data_dir = data_dir

        self._start_time = time.time()
        self._prev_state = None

        self._aero = QuanserAero(serial_port_path=serial_port_path)

    def reset(self):
        state = self._aero.send(0, 0, 0, 255, 0)
        self._start_time = time.time()
        return state, 0, False, {}

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
        reward = reward_fn(self._prev_state, action, state)

        self._prev_state = state

        return state, reward, False, {}

    def render(self, mode='human', close=False):
        pass


