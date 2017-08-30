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

simulation = True
uri = 'localhost:50051' if simulation else '192.168.1.180:50051'
simulation_data = simdata('eight-trajectory' + ('-sim' if simulation else ''))
total_time = 200 # s
n_iter = int(total_time / ctrl.dt)
error_threshould=20



def stop(stub): 
  print('Stopping...')
  stub.Configure(Telemetry(velocity=Velocity(linear=0, angular=0)))
  simulation_data.save_data()
  simulation_data.plot_graphics()
  sys.exit(0)

def run(task):
  channel = grpc.insecure_channel(uri)
  stub = GatewayStub(channel)
  stub.Configure(Telemetry(pose=Pose(x=0.0, y=0.0, heading=90.0)))

  iterations = xrange(1, n_iter + 1)
  robot = stub.Subscribe(SubscribeRequest())
  
  signal.signal(signal.SIGINT, lambda signal, frame: stop(stub))
  simulation_data.add_iterdata(iterdata(0))

  for n, telemetry, (desired_position, desired_velocity) in zip(iterations, robot, task):
    kinematic_velocity, _ = ctrl.kinematic_controller(telemetry.pose, desired_position, desired_velocity)
    stub.Configure(Telemetry(velocity=Velocity(linear=kinematic_velocity[0], angular=kinematic_velocity[1])))



    # Saving iteration data
    iter = iterdata(simulation_data.size+1) 
    iter.trajectory_position = desired_position
    iter.trajectory_velocity = desired_velocity
    iter.kinematic_velocity = kinematic_velocity
    iter.robot_heading = telemetry.pose.heading
    iter.robot_position = np.array([[telemetry.pose.x + ctrl.center_offset*np.cos(telemetry.pose.heading)], 
                                    [telemetry.pose.y + ctrl.center_offset*np.sin(telemetry.pose.heading)]])
    iter.robot_velocity = np.array([[telemetry.velocity.linear],[telemetry.velocity.angular]])
    simulation_data.add_iterdata(iter)
    
  stop(stub)

if __name__ == '__main__':
  task = tasks.eight_trajectory( dt = ctrl.dt , total_time = 80)
  run(task)
