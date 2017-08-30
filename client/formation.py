from __future__ import print_function
import controller as ctrl
import tasks as tasks
from simdata import *
import grpc
import signal
import sys
from iterdata import *
import datetime
from itertools import izip as zip
import numpy as np
from numpy import linalg
from matplotlib import interactive
import matplotlib.pyplot as plt
from robot_pb2 import Pose, Velocity, Telemetry, SubscribeRequest
from robot_pb2_grpc import GatewayStub
from scipy.linalg import block_diag


simulation = True                                                         #P3-AT                      #P3-DX
hostnames = ['localhost:50051', 'localhost:50052'] if simulation else ['192.168.1.180:50051', '192.168.1.117:50052']

dt = 100e-3
total_time = 80
n_iter = int(total_time / dt)
center_offset = 200

max_velocity = np.array([120, 120, 5.0, 120])
kinematic_gains = np.array([3, 3, 50, 3]) * 1e-3

global error_position

# max_velocity = np.array([250, 250, 1.5, 500])
# kinematic_gains = np.array([0, 0, 0, 1000])*1e-3


simulation_data = simdata('formation' + ('-sim' if simulation else ''),2)

def stop(stubs): 
  for stub in stubs:
    stub.Configure(Telemetry(velocity=Velocity(linear=0, angular=0)))
  print('Stopping...')
  simulation_data.save_data()
  simulation_data.plot_graphics()
  sys.exit(0)

def init(stubs):
  stubs[0].Configure(Telemetry(velocity=Velocity(linear=0, angular=0), pose=Pose(x=0.0, y=0.0, heading=0.0))) 
  stubs[1].Configure(Telemetry(velocity=Velocity(linear=0, angular=0), pose=Pose(x=1000.0, y=-2000.0, heading=0.0))) 


# def circular_trajectory(radius, linear, heading, distance):
#   dt = 100e-3
#   dist=distance/2
#   theta, dtheta = 0.0, linear / radius * dt
#   position1, velocity1 = np.array([0.0, 0.0]), np.array([0.0, 0.0])
#   position2, velocity2 = np.array([0.0, 0.0]), np.array([0.0, 0.0])
#   position = (position1+position2)/2
#   yield np.array([position[0], position[1], 0.0, distance]), np.array([0.0, 0.0, 0.0, 0.0])
#   while True: 
#     theta += dtheta
#     new_position1 = (radius-dist) * np.array([np.cos(theta), np.sin(theta)])
#     new_position2 = (radius+dist) * np.array([np.cos(theta), np.sin(theta)])
#     new_velocity1 = (new_position1 - position1) / dt
#     new_velocity2 = (new_position2 - position2) / dt

#     position1, velocity1 = new_position1, new_velocity1
#     position2, velocity2 = new_position2, new_velocity2
#     position = (position1+position2)/2

#     if(theta<np.pi):
#       yield np.array([position[0], position[1], theta, distance]), np.array([velocity1[0], velocity1[1], velocity2[0], velocity2[1]])
#     else:
#       yield np.array([position[0], position[1], -2*np.pi + theta, distance]), np.array([velocity1[0], velocity1[1], velocity2[0], velocity2[1]])

def fixed_point(x, y, heading, distance):
  global error_position
  error_position=500
  while True:
    if error_position < 200:
        raise StopIteration()
    yield np.array([x, y, heading, distance]), np.array([0.0, 0.0, 0.0, 0.0])

def robot_to_formation(pose1, pose2):
  x1, y1, x2, y2 = pose1.x, pose1.y, pose2.x, pose2.y
  return np.array([
    (x1 + x2) / 2.0,
    (y1 + y2) / 2.0,
    np.arctan2(y2 - y1, x2 - x1),
    np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
  ])

def inverse_jacobian2(pose1, pose2):
  x1, y1, x2, y2 = pose1.x, pose1.y, pose2.x, pose2.y
  dy, dx, p = y2-y1, x2-x1, np.sqrt((y2-y1)**2 + (x2-x1)**2)
  return np.array([
    # [1.0, 0.0,  0.5*dy, -0.5*dx/p],
    # [0.0, 1.0, -0.5*dx, -0.5*dx/p],
    # [1.0, 0.0, -0.5*dy,  0.5*dy/p],
    # [0.0, 1.0,  0.5*dx,  0.5*dy/p]
    [1.0, 0.0,  0.5*dy, -0.5*dx/p],
    [1.0, 0.0, -0.5*dy,  0.5*dx/p],
    [0.0, 1.0, -0.5*dx, -0.5*dy/p],
    [0.0, 1.0,  0.5*dx,  0.5*dy/p]
  ])

def inverse_jacobian(pose1, pose2):
  formation = robot_to_formation(pose1, pose2)
  alpha, rho = formation[2], formation[3]
  return np.array([
    [1.0, 0.0, rho*(np.sin(alpha)/2.0)  ,  -np.cos(alpha)/2.0 ],
    [0.0, 1.0, -rho*(np.cos(alpha)/2.0), -np.sin(alpha)/2.0],
    [1.0, 0.0,-rho*(np.sin(alpha)/2.0)  ,  np.cos(alpha)/2.0 ],
    [0.0, 1.0, rho*(np.cos(alpha)/2.0)  , np.sin(alpha)/2.0   ]
  ])

def inverse_kinematics(pose):
  return np.array([
    [np.cos(pose.heading),                np.sin(pose.heading)],
    [-np.sin(pose.heading)/center_offset, np.cos(pose.heading)/center_offset]
  ])

def kinematic_controller(poses, desired_formation, desired_velocity):
  ikinematics = block_diag(*[inverse_kinematics(p) for p in poses])
  ijacobian = inverse_jacobian(*poses)
  ijacobian2 = inverse_jacobian2(*poses)
 


  current_formation = robot_to_formation(*poses)
  formation_error = desired_formation - current_formation
  
  action = np.multiply(max_velocity, np.tanh(np.multiply(kinematic_gains, formation_error)))
  
  fe=np.linalg.norm(formation_error)
  print('Formation Error:', ijacobian.shape)

  return np.dot(ikinematics, np.dot(ijacobian, action.reshape(4,1))), current_formation

def run():
  global error_position
  error_position = 3000.0
  channels = [grpc.insecure_channel(c) for c in hostnames]
  stubs = [GatewayStub(c) for c in channels]  
  init(stubs)
   
  signal.signal(signal.SIGINT, lambda signal, frame: stop(stubs))

  iterations = range(n_iter)
  robots = [stub.Subscribe(SubscribeRequest()) for stub in stubs]
  trajectory = fixed_point(2000, 3000, 0.0, 1000)
  #trajectory = circular_trajectory(3000.0, 100.0, 3.14/2.0, 1000)
  simulation_data.add_iterdata(iterdata(0,2))
  error=0
  for n, (position, velocity), telemetry1, telemetry2 in zip(iterations, trajectory, *robots):
   
    poses = [t.pose for t in [telemetry1, telemetry2]]

    action, current_formation= kinematic_controller(poses, position, velocity)
    
    error_position = np.linalg.norm(current_formation[0:2]-[2000.0, 3000.0])
    #print(current_formation[2:4]) 

    stubs[0].Configure(Telemetry(velocity=Velocity(linear=action[0], angular=action[1])))
    stubs[1].Configure(Telemetry(velocity=Velocity(linear=action[2], angular=action[3])))


    #Saving iteration data
    iter=iterdata(n+1,2) 
    iter.kinematic_velocity=action.reshape(2,2)
    #iter.dinamic_velocity=velocity
    #iter.trajectory_velocity=desired_velocity
    iter.desired_position=np.array([2000.0, 3000.0])
    iter.robot_heading[0,0]=telemetry1.pose.heading
    iter.robot_heading[1,0]=telemetry2.pose.heading
    iter.robot_position[:,0]=np.array([telemetry1.pose.x + center_offset*np.cos(telemetry1.pose.heading), 
                                  telemetry1.pose.y + center_offset*np.sin(telemetry1.pose.heading)])
    iter.robot_position[:,1]=np.array([telemetry2.pose.x + center_offset*np.cos(telemetry2.pose.heading), 
                                  telemetry2.pose.y + center_offset*np.sin(telemetry2.pose.heading)])
    iter.robot_velocity[:,0]=np.array([telemetry1.velocity.linear,telemetry1.velocity.angular])

    simulation_data.add_iterdata(iter)
  
  
  stop(stubs)
  

    
if __name__ == '__main__':
  run()




