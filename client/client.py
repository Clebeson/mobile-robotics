from __future__ import print_function

import grpc
import signal
import sys

import datetime
from itertools import izip
import numpy as np
from numpy import linalg

from robot_pb2 import Pose, Velocity, Telemetry, SubscribeRequest
from robot_pb2_grpc import GatewayStub

dt = 100e-3
total_time = 40
n_iter = int(total_time / dt)
center_offset = 200

max_velocity = np.array([1000.0, 1000.0])
kinematic_gains = np.array([1, 1])*1e-3

max_acceleration = np.array([1, 1])*1e-9
dynamic_gains = np.array([1, 1])*1e-9
dynamic_parameters = np.mat([0.2604, 0.2509, -0.000499, 0.9965, 0.00263, 1.0768])

estimator_gains = np.array([1, 1])*10
estimator_parameters = np.eye(6)*1e-6

velocities = np.zeros((2, n_iter + 1))
accelerations = np.zeros((2, n_iter))

def stop(stub): 
  stub.Configure(Telemetry(velocity=Velocity(linear=0, angular=0)))
  print('Stopping...')
  sys.exit(0)

def circular_trajectory(radius, dt, linear):
  theta, dtheta = 0.0, linear / radius * dt
  position, velocity = np.array([0.0, 0.0]), np.array([0.0, 0.0])
  yield position, velocity

  while True: 
    theta += dtheta
    new_position = radius * np.array([np.cos(theta), np.sin(theta)])
    new_velocity = (new_position - position) / dt

    position, velocity = new_position, new_velocity
    yield position, velocity

def kinematic_controller(pose, desired_position, desired_velocity):
  ikinematics = np.mat([
    [np.cos(pose.heading),                np.sin(pose.heading)],
    [-np.sin(pose.heading)/center_offset, np.cos(pose.heading)/center_offset]
  ])

  current_position = np.array([pose.x, pose.y])
  position_error = desired_position - current_position

  print(current_position, linalg.norm(position_error))
  action = desired_velocity + np.multiply(max_velocity, np.tanh(np.multiply(kinematic_gains, position_error)));
  return ikinematics * action.reshape(2,1)

def dynamic_controller(velocity, desired_velocity, acceleration):
  measured_velocity = np.array([velocity.linear, velocity.angular])
  velocity_error = desired_velocity - measured_velocity

  u, w = measured_velocity[0], measured_velocity[1]
  ud, wd = desired_velocity[0], desired_velocity[1]

  dynamics = np.hstack((
    np.diag(acceleration + np.multiply(max_acceleration, np.tanh(np.multiply(dynamic_gains, velocity_error)))),
    np.mat([
      [-wd*w,       ud,     0,  0],
      [ud*w - u*wd,  0,  u*wd, wd]
    ])
  ))
  
  estimator = np.hstack((
    np.diag(acceleration + np.multiply(estimator_gains, velocity_error)),
    np.mat([
      [-w*w, u,   0, 0],
      [0,    0, u*w, w]
    ])
  )).T
  
  global dynamic_parameters 
  # dynamic_parameters += (estimator_parameters * estimator * velocity_error.reshape(2,1)).T

  return dynamics * dynamic_parameters.T

def run():
  channel = grpc.insecure_channel('localhost:50051')
  stub = GatewayStub(channel)
  stub.Configure(Telemetry(pose=Pose(x=0.0, y=0.0, heading=0.0)))

  iterations = xrange(n_iter)
  robot = stub.Subscribe(SubscribeRequest())
  trajectory = circular_trajectory(radius = 1000.0, dt = dt, linear = 250.0)
  
  signal.signal(signal.SIGINT, lambda signal, frame: stop(stub))

  for n, telemetry, (desired_position, desired_velocity) in izip(iterations, robot, trajectory):

    velocities[:,n+1] = kinematic_controller(telemetry.pose, desired_position, desired_velocity).reshape(2)
    accelerations[:,n] = (velocities[:, n+1] - velocities[:, n]) / dt
    
    velocity = dynamic_controller(telemetry.velocity, velocities[:,n+1], accelerations[:,n])  
    stub.Configure(Telemetry(velocity=Velocity(linear=velocity[0], angular=velocity[1])))
    

  stop(stub)

if __name__ == '__main__':

  run()