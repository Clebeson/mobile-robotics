from __future__ import print_function

import numpy as np
from numpy import linalg


def eight_trajectory(dt,total_time, radius=1000):
  w=250.0/1000.0
  n_iter = int(total_time / dt)
  position, velocity = np.array([0.0, 0.0]), np.array([0.0, 0.0])
  yield position, velocity
  t=0.0
  while True: 
    n_iter-=1
    if(n_iter<=0):
        raise StopIteration()

    t += 0.1
    new_position = [radius,0.0] + radius*np.array([-np.cos(w*t), np.sin(2*t*w)])
    new_velocity = (new_position - position) / dt
    
    position, velocity = new_position, new_velocity
    print(position)
    yield position, velocity



def circular_trajectory(radius, dt, linear, total_time):
  n_iter = int(total_time / dt)
  theta, dtheta = 0.0, (linear / radius) * dt
  position, velocity = np.array([0.0, 0.0]), np.array([0.0, 0.0])
  yield position, velocity
  
  while True: 
    n_iter-=1
    if(n_iter<=0):
        raise StopIteration()

    theta += dtheta
    new_position = radius * np.array([np.cos(theta), np.sin(theta)])
    velocity = (new_position - position) / dt
    position = new_position
    yield position, velocity

def generate_path(sample_rate=200, num_points=60):
    global error_position
    theta = np.radians(45)
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    position, velocity = np.array([0.0, 0.0]), np.array([0.0, 0.0])
    error_position=0
    n=0
    t=0

    while True:
        if error_position < 100:
            while True:
                t += 0.001
                new_position = 1000 * (R * np.matrix([ [t], [2*np.sin(0.5*np.pi*t)] ])).T               
                if np.linalg.norm(position-new_position) >= sample_rate:
                    break
            n += 1
            position = new_position
            print('Next point {}'.format(position.tolist()))
            if n == num_points:
                raise StopIteration()

        yield position, velocity 


def final_position(threshold=50): #total_time mm
    global error_position
    error_position = 200
    position, velocity=np.array([3000.0, 3000.0]), np.array([0.0, 0.0])
    while True:
      print(error_position)
      if error_position < threshold:
                raise StopIteration()

      yield position, velocity


