#!/usr/bin/python3

import numpy as np

class Kinematics():
    def __init__(self, r: float, L: float, B:float, kinematic_slip_angle:float = 0):
        self.r = r
        self.L = L
        self.B = B
        self.kinematic_slip_angle = kinematic_slip_angle

        self.pos_global = [0.0, 0.0]
        self.ori_global = 0

        self.last_w_Rz = 0

    def inverse(self, twist: list):
        '''- **twist**: [v_Rx, w_Rz] when v_Rx is a robot linear velocity in x axis 
                and w_Rz is a robot angular velocity in z axis'''
        
        ## This function must be overwrited by child class ##

        pass

    def forward_double_track(self, steering_angle: list, w_Wr: float, w_Wf:list):
        '''- **steering_angle**: [steering_angle_L, steering_angle_R] steering angle reference to robot x-axis
           - **w_Wr**: rear wheel angular velocity
           - **w_Wf**: [w_WfL,w_WfR] front wheel angular velocity'''
        
        # Linear Velocity Calculation
        v_Rx = self.r*np.sum(w_Wr)/2.0
        
        # Angular Velocity Calculation
        v_Wf = np.array(w_Wf) * self.r
        
        N1 = v_Wf[0]*np.cos(steering_angle[1] - self.kinematic_slip_angle)
        N2 = v_Wf[1]*np.cos(steering_angle[0] - self.kinematic_slip_angle)

        D1 =     self.L * np.sin(steering_angle[0]) * np.cos(steering_angle[1] - self.kinematic_slip_angle)
        D2 = (self.B/2) * np.cos(steering_angle[0]) * np.cos(steering_angle[1] - self.kinematic_slip_angle)
        D3 =     self.L * np.sin(steering_angle[1]) * np.cos(steering_angle[0] - self.kinematic_slip_angle)
        D4 = (self.B/2) * np.cos(steering_angle[1]) * np.cos(steering_angle[0] - self.kinematic_slip_angle)
        
        w_Rz = (N1 - N2)/(D1 + D2 + D3 + D4)
        
        return [v_Rx, w_Rz]

    def forward_single_track(self, steering_angle: float, w_Wr: float):
        '''- **steering_angle**: steering angle reference to robot x-axis
           - **w_Wr**: rear wheel angular velocity'''
        
        # Linear Velocity Calculation
        v_Rx = self.r*np.sum(w_Wr)/2.0

        # Angular Velocity Calculation
        w_Rz = (v_Rx/self.L) * np.tan(steering_angle)

        return [v_Rx, w_Rz]

    def forward_yaw_rate(self, w_Rz: float, w_Wr: float):
        '''- **steering_angle**: [steering_angle_L, steering_angle_R] steering angle reference to robot x-axis
           - **w_Rz**: robot angular velocity in z axis
           - **w_Wr**: rear wheel angular velocity'''
        
        # Linear Velocity Calculation
        v_Rx = self.r*np.sum(w_Wr)/2.0

        return [v_Rx, w_Rz]
    
    def get_pose(self, twist: list, dt: float):
        '''- **twist**: [v_Rx, w_Rz] when v_Rx is a robot linear velocity in x axis 
                and w_Rz is a robot angular velocity in z axis'''
        
        v_Rx = twist[0]
        w_Rz = twist[1]

        # Step Calculation
        dx = v_Rx * np.cos(self.ori_global + self.B + (self.last_w_Rz*dt)/2) * dt
        dy = v_Rx * np.sin(self.ori_global + self.B + (self.last_w_Rz*dt)/2) * dt
        dtheta = w_Rz * dt

        # Update
        self.pos_global[0] += dx
        self.pos_global[1] += dy
        self.ori_global    += dtheta

        self.last_w_Rz = w_Rz

        return self.pos_global, self.ori_global
    
    def reset_pose(self):
        self.pos_global = [0.0, 0.0]
        self.ori_global = 0

