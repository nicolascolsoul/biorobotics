#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Load the Pleurobot robot.
"""

from itertools import count
from pyrobolearn.simulators import Bullet
from pyrobolearn.worlds import BasicWorld
from pyrobolearn.robots import Pleurobot
import numpy as np

# Create simulator
sim = Bullet()
#box = sim.create_primitive_object(sim.GEOM_BOX, position=(0, 0, 2), mass=1, rgba_color=(1, 0, 0, 1))
# create world
world = BasicWorld(sim)
world.gravity =  np.array([0.0, 0.0, 0.0])

# create robot
robot = Pleurobot(sim)
robot.position = np.array([0, 0, 0.20])
# print information about the robot
robot.print_info()

link_id = robot.get_end_effector_ids(end_effector=5)
joint_ids = robot.joints

damping = 0.01
wrt_link_id = -1
q_idx = robot.get_q_indices(joint_ids)
kp = 50
kd = 0

sphere = world.load_visual_sphere(position=np.array([0.0, 0.0, 0.0]), radius=0.05, color=(1, 0, 0, 0.5), return_body=True)

a = 0.2
w = 0.01
# Position control using sliders
# robot.add_joint_slider()

# run simulator

##

#pos = 0;
for t in count():
    # robot.update_joint_slider()
    # robot.compute_and_draw_com_position()
    # robot.compute_and_draw_projected_com_position()

    #if pos < math.pi/2.0:
    #    pos = pos + math.pi/3600.0
    #else:
    #    pos = 0

    #print(pos)
    #robot.set_joint_positions(pos, 34)
    sphere.position = np.array([-1.0 + a * np.sin(w*t + np.pi/2.0), -0.25, np.abs(a * np.cos(w*t + np.pi/2.0))])
    x = robot.get_link_world_positions(link_id)
    # print("(xd - x) = {}".format(xd - x))

    # perform full IK
    q = robot.calculate_inverse_kinematics(link_id, position=sphere.position)

    # set the joint positions
    robot.set_joint_positions(q[q_idx], joint_ids)

    # --
    #
    # x = robot.get_link_world_positions(link_id)
    # dx = robot.get_link_world_linear_velocities(link_id)
    #
    # q = robot.get_joint_positions()
    # J = robot.get_linear_jacobian(link_id, q=q)[:, q_idx]
    # Jp = robot.get_damped_least_squares_inverse(J, damping)
    # dq = Jp.dot(kp * (sphere.position - x) - kd * dx)
    # q = q[q_idx] + dq * sim.dt
    # robot.set_joint_positions(q, joint_ids=joint_ids)

    world.step(sleep_dt=sim.dt)
