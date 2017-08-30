from matplotlib import interactive
import matplotlib.pyplot as plt
import numpy as np
import cPickle as pickle
import sys

class simdata:
    def __init__(self, name="simdata", num_robots=1):
        self.record_path='./records'
        self.data = []
        self.size = 0
        self.name=name
        self.num_robots=num_robots

    def add_iterdata(self, iterdata):
        self.data.append(iterdata)
        self.size += 1

    def get_iterdata(self, id):
        if id > self.size - 1:
            return self.data[-1]
        if id < 0 and self.size >= 0:
            return self.data[0]
        return self.data[id]

    def plot_graphics_formation(self):
        linestyles = ['-', '--', '-.', ':']
        fig = plt.figure(figsize=(10, 10))
        if self.num_robots==2:
            form_position = np.zeros((self.size,2))
            form_rho = np.zeros((self.size))
            form_alpha= np.zeros((self.size))
            form_pos_error = np.zeros((self.size))
            form_alpha_error = np.zeros((self.size))
            form_rho_error = np.zeros((self.size))
            for i, iter in zip(range(self.size), self.data):
                x1 = iter.robot_position.item(0,0)
                x2 = iter.robot_position.item(0,1)
                y1 = iter.robot_position.item(1,0)
                y2 = iter.robot_position.item(1,1)
                form_position[i,0] = (x1+x2)/2
                form_position[i,1] = (y1+y2)/2
                form_alpha[i] = np.arctan2(y2 - y1, x2 - x1)
                form_rho[i]= np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
                form_pos_error[i] = np.linalg.norm(form_position[i]-[2000.0, 3000.0])

            form_alpha_error = np.absolute(form_alpha-0)
            form_rho_error = np.absolute(form_rho-1000.0)
            plt.subplot(2, 2, 1)
            plt.title('Position')
            plt.xlabel('X [mm]')
            plt.ylabel('Y [mm]')
            pos, = plt.plot(form_position[5:-1,0], form_position[5:-1,1], linestyle='-', label='Position')
            plt.legend(handles=[pos])

            plt.subplot(2, 2, 2)
            plt.title('Alpha Error')
            plt.xlabel('# Iterations')
            plt.ylabel('Y [deg (0-180)]')
            alpha_err, = plt.plot(180*form_alpha_error[5:-1], linestyle='-', label='Alpha Error')
            plt.legend(handles=[alpha_err])

            plt.subplot(2, 2, 3)
            plt.title('Rho Error')
            plt.xlabel('# Iterations')
            plt.ylabel('[mm]')
            rho_err, = plt.plot(form_rho_error[5:-1] , linestyle='-', label='Rho Error')
            plt.legend(handles=[rho_err])

            plt.subplot(2, 2, 4)
            plt.title('Position Error')
            pos_err, = plt.plot(form_pos_error[5:-1] , linestyle='-', label='error')
            plt.legend(handles=[pos_err])
            plt.xlabel('# Iterations')
            plt.ylabel('[mm]')
            

            # for r in range(2):
                
            #     fig = plt.figure()
            #     fig.suptitle(self.name+' - Robot '+str(r+1), fontsize=20)
              
            #     rlinear = np.zeros((self.size))
            #     rangular = np.zeros((self.size))
            #     klinear = np.zeros((self.size))
            #     kangular = np.zeros((self.size))
            #     dlinear = np.zeros((self.size))
            #     dangular = np.zeros((self.size))
            #     trajectory = np.zeros((self.size))
            #     rx = np.zeros((self.size))
            #     ry = np.zeros((self.size))
            #     tx = np.zeros((self.size))
            #     ty = np.zeros((self.size))
            #     dynamic_parameters= np.zeros((self.size,6))
            #     error_position=np.zeros((self.size))
            #     error_heading=np.zeros((self.size))
            #     errx = np.zeros((self.size))
            #     erry = np.zeros((self.size))
            #     for i, iter in zip(range(self.size), self.data):
            #         klinear[i] = iter.kinematic_velocity.item(0,r)
            #         kangular[i] = iter.kinematic_velocity.item(1,r)
            #         rlinear[i] = iter.robot_velocity.item(0,r)
            #         rangular[i] = iter.robot_velocity.item(1,r)
            #         dlinear[i] = iter.dynamic_velocity.item(0,r)
            #         dangular[i] = iter.dynamic_velocity.item(1,r)
                    
            #         rx[i] = iter.robot_position.item(0,r)
            #         ry[i] = iter.robot_position.item(1,r)
            #         tx[i] = iter.desired_position.item(0)
            #         ty[i] = iter.desired_position.item(1)
            #         error_position[i]=np.linalg.norm([rx[i] - tx[i], ry[i] - ty[i] ])
            #         error_heading[i]=np.linalg.norm([iter.robot_heading - np.arctan2(6000.0 - ry[i], -8000.0 - rx[i])])
            #         dynamic_parameters[i,:]=iter.dynamic_parameters[r,:]
                    
            #     # plt.plot(linear,range(self.size))
              
            #     plt.subplot(2, 2, 1)
            #     plt.title('Velocities')
            #     plot_dynamic, = plt.plot(dlinear[10:-1], label='dynamic')
            #     plot_kinematic, = plt.plot(klinear[10:-1], label='kinematic')
            #     plot_robot, = plt.plot(rlinear[10:-1], label='robot')
            #     plt.legend(handles=[ plot_dynamic,plot_kinematic, plot_robot])

            #     plt.subplot(2, 2, 2)
            #     plt.title('Position')
            #     plot_robot, = plt.plot(rx[10:-1], ry[10:-1], label='robot')
            #     plot_trajectory, = plt.plot(tx[10:-1], ty[10:-1], label='trajectory')
            #     plt.legend(handles=[plot_robot, plot_trajectory])

            #     plt.subplot(2, 2, 3)
            #     plt.title('Position Error')
            #     position_error, = plt.plot(error_position[10:-1], label='position error')
            #     plt.legend(handles=[position_error])
                
            #     if self.num_robots==1:
            #         plt.subplot(2, 2, 4)
            #         plt.title('Velocity Error')
            #         velocity_error, = plt.plot(np.abs(v-rlinear[10:-1]), label='velocity error')
            #         plt.legend(handles=[velocity_error])
            #     else:    
            #         plt.subplot(2, 2, 4)
            #         plt.title('Heading Error')
            #         velocity_error, = plt.plot(error_heading, label='heading error')
            #         plt.legend(handles=[velocity_error])
            fig.savefig('./images/'+self.name+'.png')

            plt.show()
             

    def plot_graphics(self):
         # show graphics results
        if self.num_robots==2:
            self.plot_graphics_formation()
        else:
            fig = plt.figure(figsize=(10, 10))
           # fig.suptitle(self.name, fontsize=16)
            plt.subplots_adjust(hspace = 0.3, wspace = 0.3)
            rlinear = np.zeros((self.size))
            rangular = np.zeros((self.size))
            klinear = np.zeros((self.size))
            kangular = np.zeros((self.size))
            dlinear = np.zeros((self.size))
            dangular = np.zeros((self.size))
            trajectory = np.zeros((self.size))
            rx = np.zeros((self.size))
            ry = np.zeros((self.size))
            rv= np.zeros((self.size))
            rw = np.zeros((self.size))
            tv = np.zeros((self.size))
            tw = np.zeros((self.size))
            tx = np.zeros((self.size))
            ty = np.zeros((self.size))
            dynamic_parameters= np.zeros((self.size,6))
            error_velocity=np.zeros((self.size))
            error_position=np.zeros((self.size))
            error_heading=np.zeros((self.size))
            errx = np.zeros((self.size))
            erry = np.zeros((self.size))
            for i, iter in zip(range(self.size), self.data):
                klinear[i] = iter.kinematic_velocity.item(0)
                kangular[i] = iter.kinematic_velocity.item(1)
                rlinear[i] = iter.robot_velocity.item(0)
                rangular[i] = iter.robot_velocity.item(1)
                dlinear[i] = iter.dynamic_velocity.item(0)
                dangular[i] = iter.dynamic_velocity.item(1)
                
                rx[i] = iter.robot_position.item(0)
                ry[i] = iter.robot_position.item(1)
                tx[i] = iter.desired_position.item(0)
                ty[i] = iter.desired_position.item(1)
                
                rv[i] = iter.robot_velocity.item(0)
                rw[i] = iter.robot_velocity.item(1)
                tv[i] = iter.desired_velocity.item(0)
                tw[i] = iter.desired_velocity.item(1)
                error_velocity[i]=np.linalg.norm([rv[i] - tv[i]])
                error_position[i]=np.linalg.norm([rx[i] - tx[i], ry[i] - ty[i] ])
                error_heading[i]=np.linalg.norm([iter.robot_heading - np.arctan2(6000.0 - ry[i], -8000.0 - rx[i])])
                dynamic_parameters[i]=iter.dynamic_parameters
                
            # plt.plot(linear,range(self.size))
            
            plt.subplot(2, 2, 1)
            plt.title('Position')
            plt.xlabel('X [mm]')
            plt.ylabel('Y [mm]')

            plot_robot, = plt.plot(rx[10:-1], ry[10:-1], linestyle='-', label='robot')
            plot_trajectory, = plt.plot(tx[10:-1], ty[10:-1], linestyle=':', label='trajectory')
            plt.legend(handles=[plot_robot, plot_trajectory])
            
            if np.linalg.norm(dlinear[10:-1]) == 0:
                plt.subplot(2, 2, 2)
                plt.title('Linear Velocities')
                plt.xlabel('# Iterations')
                plt.ylabel('[mm/s]')
                # plot_trajec, = plt.plot(np.zeros(len(klinear[10:-1]))+250, linestyle='--', label='trajectory')
                plot_kinematic, = plt.plot(klinear[10:-1], linestyle='-', label='kinematic')
                plot_robot, = plt.plot(rlinear[10:-1], linestyle=':', label='robot')
                plt.legend(handles=[  plot_kinematic, plot_robot])
            else:
                plt.subplot(2, 2, 2)
                plt.title('Linear Velocities')
                plt.xlabel('# Iterations')
                plt.ylabel('[mm/s]')
                plot_dynamic, = plt.plot(dlinear[10:-1], linestyle='-',label='dynamic')
                plot_kinematic, = plt.plot(klinear[10:-1], linestyle='--', label='kinematic')
                plot_robot, = plt.plot(rlinear[10:-1], linestyle=':', label='robot')
                plt.legend(handles=[plot_dynamic, plot_kinematic, plot_robot])

            
            plt.subplot(2, 2, 3)
            plt.title('Position Error')
            plt.xlabel('# Iterations')
            plt.ylabel('[mm]')
            position_error, = plt.plot(error_position[10:-1], color='r', linestyle='-', label='position error')
            plt.legend(handles=[position_error])

            # plt.subplot(2, 2, 4)
            # plt.title('Velocity Error (Stead Error = %.2f' % np.mean(error_velocity[-150:])+')')
            # plt.xlabel('# Iterations')
            # plt.ylabel('[mm/s]')
            # velocity_error, = plt.plot(error_velocity[10:-1], color='r', linestyle='-', label='velocity error')
            # plt.legend(handles=[velocity_error])
            
            fig.savefig('./images/'+self.name+'.png')
            fig.savefig('./images/'+self.name+'.eps')

            plt.show()
    def plot_parameters(self):
       
        plt.subplots_adjust(hspace = 0.3, wspace = 0.3)
        param = np.zeros((self.size,6))
        for i, iter in zip(range(self.size), self.data):
                print(iter.dynamic_parameters)
                param[i,0] = iter.dynamic_parameters.item(0)
                param[i,1] = iter.dynamic_parameters.item(1)
                param[i,2] = iter.dynamic_parameters.item(2)
                param[i,3] = iter.dynamic_parameters.item(3)
                param[i,4] = iter.dynamic_parameters.item(4)
                param[i,5] = iter.dynamic_parameters.item(5)
        
        plt.subplot(3, 2, 1)
        plt.title('Theta 1')
        plt.plot(param[5:-1,0])
             
        plt.subplot(3, 2, 2)
        plt.title('Theta 2')
        plt.plot(param[5:-1,1])

        plt.subplot(3, 2, 3)
        plt.title('Theta 3')
        plt.plot(param[5:-1,2])

        plt.subplot(3, 2, 4)
        plt.title('Theta 4')
        plt.plot(param[5:-1,3])
             
        plt.subplot(3, 2, 5)
        plt.title('Theta 5')
        plt.plot(param[5:-1,4])

        plt.subplot(3, 2, 6)
        plt.title('Theta 6')
        plt.plot(param[5:-1,5])

        plt.show()

    

    def save_data(self):
        with open(self.record_path+'/'+self.name+".pikle",'wb') as file_name:
             simdata={'data':self.data, 'num_robots':self.num_robots}
             pickle.dump( simdata, file_name)

    def load_data(self):
        with open(self.record_path+'/'+self.name+".pikle",'rb') as file_name:
             simdata=pickle.load(file_name)
             self.data=simdata['data']
             self.num_robots=simdata['num_robots']
             self.size=len(self.data)

if __name__ == '__main__':
    sd=simdata(sys.argv[1])
    sd.load_data()
    sd.plot_graphics()
    # sd.plot_parameters()

