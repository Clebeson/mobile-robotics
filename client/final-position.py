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


dynamic_error_on = 50
def uri(simulation): 
  return 'localhost:50051' if simulation else '192.168.1.180:50051'

def stop(stub, simulation_data): 
    print('Stopping...')
    stub.Configure(Telemetry(velocity=Velocity(linear=0, angular=0)))
    simulation_data.save_data()
    simulation_data.plot_graphics()
    sys.exit(0)

def run(task, simulation):
  simulation_data = simdata('final-position' + ('-sim' if simulation else ''))
  
  channel = grpc.insecure_channel(uri(simulation))
  stub = GatewayStub(channel)
  stub.Configure(Telemetry(pose=Pose(x=0.0, y=0.0, heading=0.0)))

  robot = stub.Subscribe(SubscribeRequest())
  
  signal.signal(signal.SIGINT, lambda signal, frame: stop(stub, simulation_data))
  simulation_data.add_iterdata(iterdata(0))
  n = 0
  desired_velocity = np.array([0.0, 0.0])
  
  for telemetry, (desired_position, desired_velocity) in zip(robot, task):
    kinematic_velocity, position_error = ctrl.kinematic_controller(telemetry.pose, desired_position, desired_velocity)
    
    kinematic_acceleration = (kinematic_velocity - simulation_data.get_iterdata(simulation_data.size-1).kinematic_velocity) / ctrl.dt
    dynamic_velocity = ctrl.dynamic_controller(telemetry.velocity, kinematic_velocity, kinematic_acceleration)
    
    dynamic_velocity = kinematic_velocity

    if n > dynamic_error_on:
      dynamic_velocity = ctrl.dynamic_adapt_controller(telemetry.velocity, kinematic_velocity, kinematic_acceleration, position_error) 

    stub.Configure(Telemetry(velocity=Velocity(linear=dynamic_velocity[0], angular=dynamic_velocity[1])))
    tasks.error_position = np.linalg.norm(position_error)
    n+=1

    # Saving iteration data
    iter = iterdata(simulation_data.size+1) 
    iter.desired_position = desired_position
    iter.desired_velocity = desired_velocity*0
    iter.kinematic_velocity = kinematic_velocity
    iter.dynamic_velocity = dynamic_velocity
    iter.robot_heading = telemetry.pose.heading
    iter.robot_position = np.array([[telemetry.pose.x + ctrl.center_offset*np.cos(telemetry.pose.heading)], 
                                    [telemetry.pose.y + ctrl.center_offset*np.sin(telemetry.pose.heading)]])
    iter.robot_velocity = np.array([[telemetry.velocity.linear],[telemetry.velocity.angular]])
    iter.acceleration = kinematic_acceleration
    iter.dynamic_parameters = ctrl.dynamic_parameters
    simulation_data.add_iterdata(iter)
    
  stop(stub, simulation_data)

if __name__ == '__main__':
  task = tasks.final_position(threshold = 50)
  simulation = True if len(sys.argv) > 1 and sys.argv[1] == "sim" else False 
  run(task, simulation)
