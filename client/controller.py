from __future__ import print_function

import numpy as np
from numpy import linalg
from matplotlib import interactive
import matplotlib.pyplot as plt

#adapt-simulated-values = (0.19530318,  0.18818726, -0.00048773, 1.00222733,  0.00173159,  0.80657011)


dt = 100e-3 # s
center_offset = 200 # mm

max_velocity = np.array([250.0, 250.0])
kinematic_gains = np.array([1, 1])*1.12e-3

max_acceleration = np.array([[1], [1]])*1e-2
dynamic_gains = np.array([[1], [1]])*1e-9
dynamic_parameters_ideal = np.mat([ 0.19530273,  0.18817713, -0.00045078, 1.00294662,  0.00173049,  0.80656464])
#dynamic_parameters_ideal = np.mat([0.2604, 0.2509, -0.000499, 0.9965, 0.00263, 1.0768])
#dynamic_parameters_ideal = np.mat([ 1.24481591, 0.91870293, -0.00022601, 1.80350782, 0.00236701, 0.90890062])
global dynamic_parameters 
dynamic_parameters = dynamic_parameters_ideal * 0.75

adaptative_delay = 0 #s
#adaptative_gains = np.diag([0.2604, 0.2509, -0.000499, 0.9965, 0.00263, 1.0768])*0.5e-4
#adaptative_gains = 0.005 * np.diag([0.000005, 0.05, 0.005, 0.0001, 0.005, 0.01])
adaptative_gains = np.diag([1e-5, 5e-1, 0.01, 1e-1, 0.01, 10])*0.3e-3
adaptative_error_on = 50


def kinematic_controller(measured_pose, desired_position, desired_velocity):
  x, y, heading = measured_pose.x, measured_pose.y, measured_pose.heading

  ikinematics = np.mat([
    [np.cos(heading),                np.sin(heading)],
    [-np.sin(heading)/center_offset, np.cos(heading)/center_offset]
  ])

  current_position = np.array([x + center_offset*np.cos(heading), y + center_offset*np.sin(heading)])
  position_error = desired_position - current_position

  action = desired_velocity + np.multiply(max_velocity, np.tanh(np.multiply(kinematic_gains, position_error)))
  return ikinematics * action.reshape(2,1), position_error


# Pg. 93 [Martins]
def dynamic_controller(measured_velocity, kinematic_velocity, kinematic_acceleration):
  measured_velocity = np.array([[measured_velocity.linear], [measured_velocity.angular]])
  velocity_error = kinematic_velocity - measured_velocity
 
  action = kinematic_acceleration.reshape(2,1) + np.multiply(max_acceleration, np.tanh(np.multiply(dynamic_gains, velocity_error)))

  u, w = measured_velocity[0], measured_velocity[1]
  ud, wd = kinematic_velocity[0], kinematic_velocity[1]

  dynamics = np.hstack((
    np.diag([action[0,0], action[1,0]]),
    np.mat([
      [-wd*w,        ud,  0.0, 0.0],
      [ud*w - u*wd, 0.0, u*wd,  wd]
    ])
  ))
  
  return dynamics * dynamic_parameters_ideal.T

  # Pg. 93 [Martins]
def dynamic_adapt_controller(measured_velocity, kinematic_velocity, kinematic_acceleration, position_error):
  measured_velocity = np.array([[measured_velocity.linear], [measured_velocity.angular]])
  velocity_error = kinematic_velocity - measured_velocity
 
  action = kinematic_acceleration.reshape(2,1) + np.multiply(max_acceleration, np.tanh(np.multiply(dynamic_gains, velocity_error)))

  u, w = measured_velocity[0], measured_velocity[1]
  ud, wd = kinematic_velocity[0], kinematic_velocity[1]

  dynamics = np.hstack((
    np.diag([action[0,0], action[1,0]]),
    np.mat([
      [-wd*w,        ud,  0.0, 0.0],
      [ud*w - u*wd, 0.0, u*wd,  wd]
    ])
  ))

  # Pg. 69 [Martins]
  #if linalg.norm(position_error) > adaptative_error_on: 
  dynamic_parameters_delta = adaptative_gains * dynamics.T * velocity_error.reshape(2,1) * dt
  global dynamic_parameters 
  dynamic_parameters = np.add(dynamic_parameters, dynamic_parameters_delta.T)
  print(dynamic_parameters)

  return dynamics * dynamic_parameters.T


