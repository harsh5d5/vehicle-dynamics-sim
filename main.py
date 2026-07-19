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

