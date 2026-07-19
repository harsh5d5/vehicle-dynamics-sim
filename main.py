#          Start Program
#                │
#                ▼
#       Initialize Vehicle
#                │
#                ▼
#        Open Simulation Window
#                │
#                ▼
#       Read Keyboard Input
#       (↑ ↓ ← → Keys)
#                │
#                ▼
#    Calculate Vehicle Physics
#       (Dynamic Bicycle Model)
#                │
#                ▼
#  Update Position, Speed & Angle
#                │
#                ▼
#  Rotate Front Wheels
#                │
#                ▼
#  Draw Car
#                │
#                ▼
#  Draw IMU Axes
#                │
#                ▼
#  Update Display
#                │
#                ▼
#  Repeat at 60 FPS

import matplotlib.pyplot as plt
import numpy as np
import pygame
from pygame.locals import *
import sys, math

# Vehicle parameters
L = 2.56
Lr = L / 2.0
Lf = L - Lr
Cf = 1600.0 * 2.0   # N/rad
Cr = 1700.0 * 2.0   # N/rad
Iz = 2250.0          # kg/m2
m = 1500.0           # kg

# Control & display constants
D_STEER = 2.0
D_A = 0.75
AREA = 20

LENGTH, WIDTH, BACKTOWHEEL = 4.5, 2.0, 1.0
WHEEL_LEN, WHEEL_WIDTH, TREAD, WB = 0.3, 0.2, 0.7, 2.5

class DynamicBicycle:
    def __init__(self, x=0.0, y=0.0, theta=0.0, vx=0.01, vy=0.0, omega=0.0):
        self.xc, self.yc, self.theta = x, y, theta
        self.vx, self.vy, self.omega = vx, vy, omega
        self.delta, self.beta = 0.0, 0.0
        self.L, self.lr = L, Lr

    def step(self, a, d_delta, dt=0.2):
        if self.vx < 0.1:
            self._kinematic(a, d_delta, dt)
        else:
            self._dynamic(a, d_delta, dt)

    def _update_steering(self, d_delta, dt):
        if d_delta != 0:
            self.delta += d_delta * dt
        else:
            self.delta -= self.delta * 4.0 * dt  # self-centering
        self.delta = np.clip(self.delta, -0.5, 0.5)

    def _kinematic(self, a, d_delta, dt):
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L)
        self.xc += self.vx * np.cos(self.theta) * dt
        self.yc += self.vx * np.sin(self.theta) * dt
        self.theta += self.vx * np.cos(self.beta) * np.tan(self.delta) / self.L * dt
        self.vx += a * dt - (self.vx * 0.4) * dt
        self._update_steering(d_delta, dt)
        self.vx = np.clip(self.vx, -20, 20)
        # Feed-forward for smooth kinematic→dynamic transition
        self.vy = self.vx * np.sin(self.beta)
        self.omega = self.vx * np.cos(self.beta) * np.tan(self.delta) / self.L
