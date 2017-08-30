
import numpy as np
class iterdata:
    
    def __init__(self, iter, num_robots=1):
        self.kinematic_velocity=np.zeros((2,num_robots))
        self.dynamic_velocity=np.zeros((2,num_robots))
        self.desired_velocity=np.zeros((2))
        self.desired_position=np.zeros((2))
        self.robot_velocity=np.zeros((2,num_robots))
        self.robot_position=np.zeros((2,num_robots))
        self.robot_heading=np.zeros((num_robots,1))
        self.acceleration=np.zeros((2,num_robots))
        self.dynamic_parameters=np.zeros((num_robots,6))
        self.iteration=iter
        self.num_robots=num_robots
        
        