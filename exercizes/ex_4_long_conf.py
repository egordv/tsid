# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import os

import numpy as np

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

DATA_FILE_LIPM = "romeo_walking_traj_long_lipm.npz"
DATA_FILE_TSID = "romeo_walking_traj_long_tsid.npz"

# robot parameters
# ----------------------------------------------
filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + "/../models/romeo"
urdf = path + "/urdf/romeo.urdf"
srdf = path + "/srdf/romeo_collision.srdf"
lxp = 0.07  # foot length in positive x direction
lxn = 0.07  # foot length in negative x direction
lyp = 0.05  # foot length in positive y direction
lyn = 0.05  # foot length in negative y direction
lz = 0.07  # foot sole height with respect to ankle joint
mu = 0.3  # friction coefficient
fMin = 5.0  # minimum normal force
fMax = 1000.0  # maximum normal force
rf_frame_name = "RAnkleRoll"  # right foot frame name
lf_frame_name = "LAnkleRoll"  # left foot frame name
contactNormal = np.matrix(
    [0.0, 0.0, 1.0]
).T  # direction of the normal to the contact surface

# configuration for LIPM trajectory optimization
# ----------------------------------------------
alpha = 10 ** (2)  # CoP error squared cost weight
beta = 0  # CoM position error squared cost weight
gamma = 10 ** (-1)  # CoM velocity error squared cost weight
h = 0.58  # fixed CoM height
g = 9.81  # norm of the gravity vector
foot_step_0 = np.array([0.0, -0.096])  # initial foot step position in x-y
dt_mpc = 0.2  # sampling time interval
T_step = 0.8  # time needed for every step
step_length = 0.2  # fixed step length
step_height = 0.05  # fixed step height
nb_steps = 6  # number of desired walking steps

# configuration for TSID
# ----------------------------------------------
dt = 0.002  # controller time step
T_pre = 1.0  # simulation time before starting to walk
T_post = 2.0  # simulation time after walking

w_com = 1.0  # weight of center of mass task
w_foot = 1e-1  # weight of the foot motion task
w_posture = 1e-4  # weight of joint posture task
w_forceRef = 1e-5  # weight of force regularization task
w_torque_bounds = 0.0  # weight of the torque bounds
w_joint_bounds = 0.0

tau_max_scaling = 1.55  # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0  # proportional gain of contact constraint
kp_foot = 10.0  # proportional gain of contact constraint
kp_com = 10.0  # proportional gain of center of mass task
kp_posture = 1.0  # proportional gain of joint posture task

# configuration for viewer
# ----------------------------------------------
PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 20  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [
    3.578777551651001,
    1.2937744855880737,
    0.8885031342506409,
    0.4116811454296112,
    0.5468055009841919,
    0.6109083890914917,
    0.3978860676288605,
]
