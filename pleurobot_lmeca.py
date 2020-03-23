#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Load the Pleurobot robot.
"""

from itertools import count
from pyrobolearn.simulators import Bullet
from pyrobolearn.worlds import BasicWorld
from pyrobolearn.robots import Pleurobot
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import fsolve

# Create simulator
sim = Bullet()
#box = sim.create_primitive_object(sim.GEOM_BOX, position=(0, 0, 2), mass=1, rgba_color=(1, 0, 0, 1))
# create world
world = BasicWorld(sim)
world.gravity =  np.array([0.0, 0.0, 0.0])

# create robot
robot = Pleurobot(sim)
robot.position = np.array([0, 0, 0.21])
# print information about the robot
robot.print_info()

dt = 1./240
link_id = robot.get_end_effector_ids(end_effector=5)
joint_ids = robot.joints

wrt_link_id = -1
q_idx = robot.get_q_indices(joint_ids)
xd = np.array([-1.0, -0.25, 0.0])
sphere = world.load_visual_sphere(position=xd, radius=0.05, color=(1, 0, 0, 0.5), return_body=True)

# Nicolas new part :
stride_length = 0.20
stride_height = 0.10
start_angle = 50.0 * (np.pi/180)
landing_angle = - 45.0 * (np.pi/180)
# curve type : ax⁴ + bx³ + cx² + dx + e = y
# passing by (0;0) so,
e = 0
# passing by (stride_length, 0) so,
# EQ1 : a*stride_length^4 + b*stride_length^3 + c*stride_length^2 + d*stride_length + e = 0
#passing by (stride_length/2, stride_height)
# EQ2 : a*(stride_length/2)^4 + b*(stride_length/2)^3 + c*(stride_length/2)^2 + d*(stride_length/2) + e - stride_height = 0
# slope at (0;0) = start_angle :
# EQ3 : e = start_angle
d = np.tan(start_angle)
# slope at (stride_length;0) = landing_angle
# EQ4 : 4*a*stride_length^3 + 3*b*stride_length^3 + 2*c*stride_length + d - anding_angle = 0

def equations(p):
    a, b, c = p
    return (a*stride_length**4 + b*stride_length**3 + c*stride_length**2 + d*stride_length,
    a*(stride_length/2)**4 + b*(stride_length/2)**3 + c*(stride_length/2)**2 + d*(stride_length/2) - stride_height,
    4*a*stride_length**3 + 3*b*stride_length**2 + 2*c*stride_length + d - np.tan(landing_angle))

a, b, c =  fsolve(equations, (1, 1, 1))

x_plot = np.linspace(0.0, stride_length, num=1000)
y_plot = a * np.power(x_plot, 4) + b * np.power(x_plot, 3) + c * np.square(x_plot) + d * x_plot + e

plt.plot(0.0, 0.0, 'rs')
plt.plot(0.0 + stride_length, 0.0, 'rs')
plt.plot(stride_length/2, stride_height, 'rs')
plt.plot(x_plot, y_plot)
plt.show()

x_t = 0.0;
y_t = 0.0;
v = 0.1;
k_v = 1; # Speed sign
# run simulator
for t in count():
    if x_t <= stride_length and x_t >= 0.0:
        x_t = x_t + v * k_v * dt
    elif x_t > stride_length:
        k_v = -1
        x_t = stride_length
    elif x_t < 0.0:
        k_v = 1
        x_t = 0.0

    y_t = a * x_t**4 + b * x_t**3 + c * x_t**2 + d * x_t + e


    sphere.position = np.array([x_t, 0.0, y_t])
    x = robot.get_link_world_positions(link_id)
    q = robot.calculate_inverse_kinematics(link_id, position=sphere.position)

    # set the joint positions
    robot.set_joint_positions(q[q_idx], joint_ids)

    # for x in joint_ids:
    #     #print(x)
    #     n = robot.get_link_names(x)
    #     if n != "link16" and n != "link17" and n != "link18" and n != "link19":
    #         print()
    #         #robot.set_joint_positions(0.0, x)
    #     else:
    #         print("Success")
    #
    # print("--")

    robot.position = np.array([0.9, 0.4, 0.10])
    world.step(sleep_dt=dt)
