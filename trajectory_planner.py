import numpy as np 
import time
from math import *

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
    
    def set_initial_wp(self):
        self.initial_wp = self.final_wp
        return self.initial_wp

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint
        return self.final_wp

    def go(self, max_speed = 2.5):
        pass

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        wp_diff = []
        for i in range(self.num_joints):
            wp_diff.append(abs(final_wp[i]-initial_wp[i]))
        max_wp_diff = max(wp_diff)
        T = max_wp_diff/max_speed
        return T

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        ts = 0.0
        tf = T
        N = int((tf-ts)/self.dt)+1
        t = np.linspace(ts, tf, int(N), endpoint=True)
        spline_pts = [[]]*self.num_joints
        spline_velos = [[]]*self.num_joints
        for j in range(self.num_joints):
            Qs = [self.initial_wp[j]]
            Qf = [self.final_wp[j]]
            Vs = [0]
            Vf = [0]
            M = np.array([[1, ts, ts**2, ts**3],
                          [0, 1,  2*ts, 3*(ts**2)],
                          [1, tf, tf**2, tf**3],
                          [0, 1,  2*tf, 3*(tf**2)]])
            QV = np.array([Qs, Vs, Qf, Vf])
            QV = QV.T
            A = np.linalg.inv(M).dot(QV[0])
            Qt = A[0]+A[1]*t+A[2]*np.power(t,2)+A[3]*np.power(t,3);
            spline_pts[j]=Qt.tolist()
            Vt = A[1]+2*A[2]*t+3*A[3]*np.power(t,2)
            spline_velos[j]=Vt.tolist()
        plan_pts = np.array(spline_pts).T.tolist()
        plan_velos = np.array(spline_velos).T.tolist()
        return plan_pts, plan_velos

    def execute_plan(self, plan_pts, plan_velos, look_ahead=8):
        for i in range(len(plan_pts)):
            self.rexarm.set_positions(plan_pts[i])
            self.rexarm.set_speeds(plan_velos[i])
            self.rexarm.pause(self.dt)