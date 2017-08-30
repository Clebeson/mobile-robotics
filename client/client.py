from __future__ import print_function

import grpc
import signal
import sys
from iterdata import *
from simdata import *
import datetime
from itertools import izip as zip
import numpy as np
from numpy import linalg
from matplotlib import interactive
import matplotlib.pyplot as plt


from robot_pb2 import Pose, Velocity, Telemetry, SubscribeRequest
from robot_pb2_grpc import GatewayStub


dt = 100e-3
total_time = 40
n_iter = int(total_time / dt)
center_offset = 200

max_velocity = np.array([250.0, 250.0])
kinematic_gains = np.array([1, 1])*1.12e-3

max_acceleration = np.array([1, 1])*0.2e-2
dynamic_gains = np.array([7, 1])*2e-9

dynamic_parameters2 = np.mat([0.2604, 0.2509, -0.000499, 0.9965, 0.00263, 1.0768])*0.75

dynamic_parameters = np.mat([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
estimator_gains = np.array([1, 100])*6e-2
estimator_parameters = 0.005*np.matrix(np.diag([0.005, 0.05, 0.0005, 0.01, 0.005, 0.01]))
leakage_parameters = 0.005*np.diag([0.005, 0.05, 0.0005, 0.01, 0.005, 0.01])
simulation_data=simdata()

#velocities = np.zeros((2, n_iter + 1))
#accelerations = np.zeros((2, n_iter))
global next_point
       
       
def stop(stub): 
  stub.Configure(Telemetry(velocity=Velocity(linear=0, angular=0)))
  print('Stopping...')
  sys.exit(0)

def generate_path():
    global next_point
    import matplotlib.pyplot as plt # For ploting
    theta = np.radians(45)
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    T = np.arange(0.0, 6.0, 0.1)  
    S = 2*np.sin(0.5*np.pi*T) 
    points= 1000*(R * np.matrix('{};{}'.format(T,S))).T
    velocity = np.array([0.0, 0.0])
    for position in points[1:-1,:]:
      next_point=False
      print(position)
      while not next_point:
        yield position, velocity 

def final_position():
    position, velocity=np.array([3000.0, 3000.0]), np.array([0.0, 0.0])
    while True:
      yield position, velocity



     
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

  current_position = np.array([pose.x + center_offset*np.cos(pose.heading), pose.y + center_offset*np.sin(pose.heading)])
  position_error = desired_position - current_position
  
  if linalg.norm(position_error) < 100.0:
    global next_point
    next_point=True
    print('next')
    return np.array([[100.0], [100.0]])

  # print(current_position, linalg.norm(position_error))
  action = desired_velocity + np.multiply(max_velocity, np.tanh(np.multiply(kinematic_gains, position_error)))
  return ikinematics * action.reshape(2,1)

def dynamic_controller(velocity, desired_velocity, acceleration):
  measured_velocity = np.array([velocity.linear, velocity.angular])
  velocity_error = desired_velocity[0] - measured_velocity


  u, w = measured_velocity[0], measured_velocity[1]
  ud, wd = desired_velocity[0], desired_velocity[1]
  ku, lu, kw, lw = 4.0, 0.2, 4.0, 4.0
 
  acc=acceleration.reshape(1,2) + np.multiply(max_acceleration, np.tanh(np.multiply(dynamic_gains, velocity_error)))
  #acc=acceleration.reshape(1,2) + np.multiply([lu , lw], np.tanh(np.multiply([ku/lu, kw/lw], velocity_error)))

  dynamics = np.hstack((
    np.diag([acc[0,0],acc[0,1]]),
    np.mat([
      [-wd*w,       ud,     0.0,  0.0],
      [ud*w - u*wd,  0.0,  u*wd, wd]
    ])
  ))

   
  # acc=acceleration + np.multiply(estimator_gains, velocity_error)
  # estimator = np.hstack((
  # np.diag([acc[0,0],acc[0,1]]),
  # np.mat([
  # [-w*w, u,   0, 0],
  #  [0,    0, u*w, w]
  # ])
  # )).T

   
  global dynamic_parameters 

  dynamic_parameters_dot =  (
                           ((estimator_parameters * dynamics.T) * velocity_error.reshape(2,1)
                           #-(estimator_parameters *leakage_parameters*dynamic_parameters.T)
                           )/dt).T

  #global dynamic_parameters2
  dynamic_parameters=np.add(dynamic_parameters, dynamic_parameters_dot)
  #dynamic_parameters += dynamic_parameters_dot
  #print(dynamic_parameters2-dynamic_parameters)

  return dynamics * dynamic_parameters.T


def run(task, time=0.0):
  
  
  if time==0.0:
     n_iter=int(80.0/dt)
  else:
     n_iter=int(time/dt)


  channel = grpc.insecure_channel('localhost:50051')
  # channel = grpc.insecure_channel('192.168.1.143:50051')
  stub = GatewayStub(channel)
  stub.Configure(Telemetry(pose=Pose(x=0.0, y=0.0, heading=0.0)))

  iterations = xrange(n_iter)
  robot = stub.Subscribe(SubscribeRequest())
  
  signal.signal(signal.SIGINT, lambda signal, frame: stop(stub))
  simulation_data.add_iterdata(iterdata(0))

  for n, telemetry, (desired_position, desired_velocity) in zip(iterations, robot, task):
      kinematic_vel= kinematic_controller(telemetry.pose, desired_position, desired_velocity)
      if linalg.norm(kinematic_vel) ==0.0:
        break
      acc= (kinematic_vel - simulation_data.get_iterdata(n).kinematic_velocity) / dt
      dinamic_vel = dynamic_controller(telemetry.velocity,kinematic_vel, acc) 
      velocity=kinematic_vel
      print('{} {}'.format(velocity[0], velocity[1]))
      stub.Configure(Telemetry(velocity=Velocity(linear=velocity[0], angular=velocity[1])))
      global dynamic_parameters 
      
      #Saving iteration data
      iter=iterdata(n+1) 
      iter.kinematic_velocity=kinematic_vel
      iter.dinamic_velocity=dinamic_vel
      iter.trajectory_velocity=desired_velocity
      iter.trajectory_position=desired_position
      iter.robot_heading=telemetry.pose.heading

      iter.robot_position=np.array([[telemetry.pose.x + center_offset*np.cos(telemetry.pose.heading)], 
                                    [telemetry.pose.y + center_offset*np.sin(telemetry.pose.heading)]])

      iter.robot_velocity=np.array([[telemetry.velocity.linear],[telemetry.velocity.angular]])
      iter.acceleration=acc
      iter.dynamic_parameters=dynamic_parameters
      simulation_data.add_iterdata(iter)
    
    #simulation_data.dynamic_graphics(plt)
    #plt.pause(0.001)
  
  stub.Configure(Telemetry(velocity=Velocity(linear=0, angular=0)))
  
  simulation_data.save_data()
  simulation_data.plot_graphics()
  

  #stop simulation
  stop(stub)

if __name__ == '__main__':
  path=generate_path()
  # path=final_position()
  # trajectory = circular_trajectory(radius = 1000.0, dt = dt, linear = 250.0)
  run(path,300.0)

  