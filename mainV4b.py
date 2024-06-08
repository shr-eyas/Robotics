'''
There might be some issues with the way error is calculated

Trying to correct for it.
'''

import time
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

from sympy import symbols   
from aruco import ArUco
from utilities import Grasp

from leap_hand_utils.dynamixel_client import *
from Motor_control_lib import Multimotor_control


utils = Grasp()
aruco = ArUco(marker_length=0.025, base_marker_id=0, object_marker_id=1)
 
# kP = 600   # Set Kp of motors
# kI = 0     # Set Ki of motors
# kD = 200   # Set Kd of motors
# curr_lim = 350 # Set maximum current limit

# # Define motor IDs
# robotIDs = [11, 13, 21, 23]
# redundantIDs = [12, 14, 22, 24]
# robot = Multimotor_control(IDs = robotIDs)
# redundantRobot = Multimotor_control(IDs = redundantIDs)

# #Activate the torque -6.2         -6.20
# robot.torque_activate(robotIDs)
# redundantRobot.torque_activate(redundantIDs)

# # Set Parameters of the motor
# robot.set_motor_propotional_gain(robotIDs,np.ones(len(robotIDs)) * kP)   
# robot.set_motor_derivative_gain(robotIDs,np.ones(len(robotIDs)) * kD) 
# robot.set_motor_integrator_gain(robotIDs,np.ones(len(robotIDs)) * kI)
# robot.set_motor_max_current(robotIDs,np.ones(len(robotIDs)) * curr_lim)

# redundantRobot.set_motor_propotional_gain(redundantIDs,np.ones(len(redundantIDs)) * kP)   
# redundantRobot.set_motor_derivative_gain(redundantIDs,np.ones(len(redundantIDs)) * kD) 
# redundantRobot.set_motor_integrator_gain(redundantIDs,np.ones(len(redundantIDs)) * kI)
# redundantRobot.set_motor_max_current(redundantIDs,np.ones(len(redundantIDs)) * curr_lim)

dt = 0.1
to = 0
tf = 1
a = 0.0375 # 37.5 mm
Kt = 0.35

timer = np.arange(to, tf + dt, dt)
trials = 1

lambda_ILC = 0.7
gamma_ILC = 0.1

t = symbols('t')

xd, xdotd, xddotd = utils.trajectory_planner(to, tf, 0, 0, 0, 0)
yd, ydotd, yddotd = utils.trajectory_planner(to, tf, 0.17, 0, 0.15, 0)
th, thd, thdd = utils.trajectory_planner(to, tf, 0, 0, 0, 0)

x = np.array([float(xd.subs(t, time)) for time in timer])
y = np.array( [float(yd.subs(t, time)) for time in timer])
theta = np.array([float(th.subs(t, time)) for time in timer])

desX = np.vstack((x, y, theta))

print(desX)
bodyF = [np.zeros((3, len(timer) - 1)) for _ in range(trials)]
error = [None] * trials
G = [[None] * (len(timer) - 1) for _ in range(trials)]
fingerF = [[np.zeros(4)] * (len(timer) - 1) for _ in range(trials)]
tau = [[None] * (len(timer) - 1) for _ in range(trials)]
# handJacobian = [[None] * (len(timer) - 1) for _ in range(trials)]

positions = []
trial_positions = []

for i in range(trials):
    # print('Going to home position')
    # redundantRobot.set_operating_mode(redundantIDs, np.ones(len(redundantIDs))*3)
    # redundantRobot.set_goal_position([np.pi, np.pi, np.pi, np.pi], redundantIDs)
    # robot.set_operating_mode(robotIDs, np.ones(len(robotIDs))*3)
    # robot.torque_activate(robotIDs)
    q1, q2 = utils.IK_2R_elbow_up(-80, 174, 103, 93)
    q3, q4 = utils.IK_2R_elbow_down(80, 174, 103, 93)
    offset = np.array([np.pi/2, np.pi, np.pi/2, np.pi])
    # robot.set_goal_position((np.radians([q1, q2, q3, q4]) + offset), robotIDs)
    # time.sleep(5)
    # robot.torque_de_activate(robotIDs)
    
    # robot.set_operating_mode(robotIDs, np.ones(len(robotIDs))*0)
    # robot.torque_activate(robotIDs)

    for j in range(len(timer) - 1):
        
        Q = aruco.get_marker_info()
        G[i][j] = utils.grasp_matrix(Q[2], a)

        resultantF = bodyF[i]
        Fx = resultantF[0, j]
        Fy = resultantF[1, j]
        Mz = resultantF[2, j]

        # redundantRobot.set_goal_position([np.pi, np.pi, np.pi, np.pi], redundantIDs)

        # actual_positions = robot.read_positions()
        # phi1 = [np.radians(actual_positions[0]), np.radians(actual_positions[1])]
        # phi2 = [np.radians(actual_positions[2]), np.radians(actual_positions[3])]
  
        # Jh = utils.hand_jacobian(phi1, phi2, [0.103, 0.093], [0.103, 0.093], Q[2], 0, 0)
        # handJacobian = np.array(Jh, dtype=float)

        fingerF[i][j] = np.dot((np.identity(4) - np.dot(np.linalg.pinv(np.array(G[i][j], dtype=float)), \
                                                        np.array(G[i][j], dtype=float))), np.array([[1],[1],[1],[1]])) \
                                                        + np.dot(np.linalg.pinv(np.array(G[i][j], dtype=float)), np.array([[Fx], [Fy], [Mz]]))

        # handJacobianInv = np.linalg.pinv(handJacobian)
        # tau[i][j] = np.dot(handJacobianInv, fingerF[i][j])
        # goalCurrent = tau[i][j] / Kt
        # goalCurrent[1] = -goalCurrent[1]
        # goalCurrent[3] = -goalCurrent[3]

        # print(f'i: {i}, j:{j}, bodyF: {bodyF[i]}')
 
        # robot.set_goal_current(goalCurrent, robotIDs)
        # print(f'i: {i}, j:{j}, Finger Force: {fingerF[i][j]}, Fx: {resultantF[0, j]}, Fy: {resultantF[1, j]}, Mz: {resultantF[2, j]}')
        # print(f'i: {i}, j:{j}, G[i][j]: {G[i][j]}')

        trial_positions.append([Q[0], Q[1], Q[2]])
    print(trial_positions)
    positions.append(trial_positions)
    
    # error[i] = np.array(desX[:, :len(timer) - 1]) - np.array(positions[i]).T
    # print(f"i: {i}, Error: {error[i]*1000}")
    # bodyF[i + 1] = lambda_ILC * bodyF[i] + gamma_ILC * error[i]
    # robot.torque_de_activate(robotIDs)

desX is: 
[[0.      0.      0.      0.      0.      0.      0.      0.      0.
  0.      0.     ]
 [0.17    0.16944 0.16792 0.16568 0.16296 0.16    0.15704 0.15432 0.15208
  0.15056 0.15   ]
 [0.      0.      0.      0.      0.      0.      0.      0.      0.
  0.      0.     ]]


trial_positions is this:
[[0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155], 
 [0.0007, 0.1762, -0.0219], 
 [0.0008, 0.1762, -0.0155], 
 [0.0008, 0.1762, -0.0155]]