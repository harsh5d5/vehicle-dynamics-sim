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
#       (↑ ↓ ← → Keys for the moving)
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

    def _dynamic(self, a, d_delta, dt):
        self.xc += self.vx * math.cos(self.theta) * dt - self.vy * math.sin(self.theta) * dt
        self.yc += self.vx * math.sin(self.theta) * dt + self.vy * math.cos(self.theta) * dt
        self.theta += self.omega * dt
        vx_safe = max(abs(self.vx), 0.1) * np.sign(self.vx)
        Ffy = -Cf * math.atan2((self.vy + Lf * self.omega) / vx_safe - self.delta, 1.0)
        Fry = -Cr * math.atan2((self.vy - Lr * self.omega) / vx_safe, 1.0)
        self.vx += (a - Ffy * math.sin(self.delta) / m + self.vy * self.omega) * dt - (self.vx * 0.4) * dt
        self.vy += (Fry / m + Ffy * math.cos(self.delta) / m - self.vx * self.omega) * dt
        self.omega += (Ffy * Lf * math.cos(self.delta) - Fry * Lr) / Iz * dt
        self._update_steering(d_delta, dt)
        self.vx = np.clip(self.vx, -20, 20)

    def reset(self):
        self.xc = self.yc = self.theta = self.vy = self.omega = self.delta = 0.0
        self.vx, self.beta = 0.01, 0.0


def _rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, s], [-s, c]])

def plot_car(x, y, yaw, steer=0.0):
    outline = np.array([[-BACKTOWHEEL, LENGTH-BACKTOWHEEL, LENGTH-BACKTOWHEEL, -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH/2, WIDTH/2, -WIDTH/2, -WIDTH/2, WIDTH/2]])
    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH-TREAD, -WHEEL_WIDTH-TREAD, WHEEL_WIDTH-TREAD, WHEEL_WIDTH-TREAD, -WHEEL_WIDTH-TREAD]])
    rr_wheel = np.copy(fr_wheel)
    fl_wheel = np.copy(fr_wheel); fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel); rl_wheel[1, :] *= -1

    R_yaw, R_steer = _rot(yaw), _rot(steer)
    fr_wheel = (fr_wheel.T @ R_steer).T; fl_wheel = (fl_wheel.T @ R_steer).T
    fr_wheel[0, :] += WB; fl_wheel[0, :] += WB
    for w in [fr_wheel, fl_wheel]:
        w[:] = (w.T @ R_yaw).T
    for part in [outline, rr_wheel, rl_wheel]:
        part[:] = (part.T @ R_yaw).T
    for part in [outline, fr_wheel, rr_wheel, fl_wheel, rl_wheel]:
        part[0, :] += x; part[1, :] += y
        plt.plot(part[0, :].flatten(), part[1, :].flatten(), '-k')
    plt.plot(x, y, "*")

def plot_imu_axes(x, y, yaw):
    L_ax = 2.0
    plt.plot([x, x + L_ax * np.cos(yaw)], [y, y + L_ax * np.sin(yaw)], 'k--')
    plt.plot([x, x + L_ax * np.cos(yaw + np.pi/2)], [y, y + L_ax * np.sin(yaw + np.pi/2)], 'm--')


def loop():
    pygame.init()
    window = pygame.display.set_mode((800, 800))
    pygame.display.set_caption("2D Bicycle Model")
    clock = pygame.time.Clock()
    dt = 1 / 60.0
    last_time = pygame.time.get_ticks()

    plt.ion()
    plt.figure(figsize=(8, 8))
    trail_x, trail_y = [], []

    car = DynamicBicycle()

    while True:
        a, d_steer = 0, 0
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit(); sys.exit()

        keys = pygame.key.get_pressed()
        if keys[K_RIGHT] or keys[K_d]: d_steer = -D_STEER
        if keys[K_LEFT]  or keys[K_a]: d_steer =  D_STEER
        if keys[K_UP]    or keys[K_w]: a =  D_A
        if keys[K_DOWN]  or keys[K_s]: a = -D_A * 2
        if keys[K_r]:
            car.reset(); trail_x.clear(); trail_y.clear()

        now = pygame.time.get_ticks()
        if (now - last_time) / 1000.0 >= dt:
            car.step(a, d_steer, dt)
            last_time = now
            trail_x.append(car.xc); trail_y.append(car.yc)

        clock.tick(60)

        plt.gca().clear()
        plt.xlim([car.xc - AREA, car.xc + AREA])
        plt.ylim([car.yc - AREA, car.yc + AREA])
        plt.grid(True)
        if len(trail_x) > 1:
            plt.plot(trail_x[-300:], trail_y[-300:], 'b--', alpha=0.5)
        plot_car(car.xc, car.yc, car.theta, car.delta)
        plot_imu_axes(car.xc, car.yc, car.theta)

        mode = "Kinematic" if car.vx < 0.1 else "Dynamic"
        plt.title(f"Bicycle Model [{mode}] | {car.vx*3.6:.1f} km/h | δ={math.degrees(car.delta):.1f}°")
        plt.draw(); plt.pause(0.001)
        window.fill((30, 30, 30)); pygame.display.flip()

loop()
