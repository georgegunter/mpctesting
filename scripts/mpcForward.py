#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int16, String, Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import time


# For CASADI:
from casadi import *


import numpy as np
import pandas as pd
import json

# velocity_topic = "/car/state/vel_x"
# sport_mode_topic = "car/state/sport_mode"
# eco_mode_topic = "car/state/eco_mode"
# gantry_topic = "/vsl/latest_gantry"
# vsl_set_speed_topic = "/vsl/set_speed"
# distance_lines_topic="/acc/set_distance"

lead_x_topic = "/lead_dist"
lead_rv_topic = "/rel_vel"
velocity_topic = "/car/state/vel_x"


max_speed = 32 ##32 m/s is 71.6 mph
velocity = None
lead_x=252
lead_rv=0

mpc_cmd_accel = None

def velocity_callback(data):
    global velocity
    velocity = data.data

def lead_x_callback(data):
    global lead_x
    lead_x = data.data

def lead_rv_callback(data):
    global lead_rv
    lead_rv = data.data


def recursivePop():
    global radar_state
    if len(radar_state[3]) !=0:
        oldest_time = radar_state[3].pop(0)
        #print(oldest_time, time.time())
        if abs(oldest_time - time.time()) > 0.1:
            radar_state[0].pop(0) #pop the oldest stuff
            radar_state[1].pop(0)
            radar_state[2].pop(0)
            recursivePop()
        else:
            radar_state[3].insert(0,oldest_time)
    else:
        return



class MPC_min_forward_spacing_timegap:
    def __init__(self,tg_min,s_min,v_des,T=2.0,N=20,alpha_smooth=0.1,alpha_v_cntrl=0.1,use_all_control=False):

        self.v_des = v_des
        self.dt = T/N
        self.s_min = s_min

        self.T = T # Prediction time horizon
        self.N = N # number of control intervals

        self.alpha_smooth = alpha_smooth # penalty for strong control commands:
        self.alpha_v_cntrl = alpha_v_cntrl

        # Declare model variables for prediction:
        self.x1 = MX.sym('x1')
        self.x2 = MX.sym('x2')
        self.x = vertcat(self.x1, self.x2)
        self.u = MX.sym('u')

        self.F = self.get_integrator()

    ###### Set up MPC calc functions: ######

    def get_integrator(self):

        # optimize over a predicted leader trajectory:

        # x1 = v
        # x2 = p

        # dynamical model equations:
        xdot = vertcat(self.u, self.x1) #double integrator

        # Lagrangian definition to optimize over:
        L = self.alpha_v_cntrl*(self.x1-self.v_des)**2 + self.alpha_smooth*(self.u**2)

        # Formulate discrete time dynamics
        if False:
           # CVODES from the SUNDIALS suite
           dae = {'x':self.x, 'p':self.u, 'ode':xdot, 'quad':L}
           F = integrator('F', 'cvodes', dae, 0, self.T/self.N)
        else:
           # Fixed step Runge-Kutta 4 integrator
           M = 4 # RK4 steps per interval
           DT = self.T/self.N/M
           f = Function('f', [self.x, self.u], [xdot, L])
           X0 = MX.sym('X0', 2)
           U = MX.sym('U')
           X = X0
           Q = 0
           for j in range(M):
               k1, k1_q = f(X, U)
               k2, k2_q = f(X + DT/2 * k1, U)
               k3, k3_q = f(X + DT/2 * k2, U)
               k4, k4_q = f(X + DT * k3, U)
               X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
               Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
           F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

           return F

    def solve_cf_dms_smin(self,x_init,v_l_vals,p_l_vals):

        # Start with an empty NLP
        w=[]
        w0 = []
        lbw = []
        ubw = []
        J = 0
        g=[]
        lbg = []
        ubg = []

        # "Lift" initial conditions
        Xk = MX.sym('X0', 2)
        w += [Xk]
        lbw += x_init
        ubw += x_init
        w0 += x_init

        # Formulate the NLP
        for k in range(self.N):
            # New NLP variable for the control
            Uk = MX.sym('U_' + str(k))
            w   += [Uk]
            lbw += [-5]
            ubw += [5]
            w0  += [0]

            # Integrate till the end of the interval
            Fk = self.F(x0=Xk, p=Uk)
            Xk_end = Fk['xf']
            J=J+Fk['qf']

            # New NLP variable for state at end of interval
            Xk = MX.sym('X_' + str(k+1), 2)
            w   += [Xk]

            # x1 = v
            # x2 = p

            # Encode spacing constraint:
            p_max = p_l_vals[k] - self.s_min

            lbw += [0.0, 0] 
            ubw += [30.0,  p_max]
            w0  += [0, 0]

            # Add equality constraint
            g   += [Xk_end-Xk]
            lbg += [0, 0]
            ubg += [0, 0]

            # minimum time-gap constraint:
            g += [(p_l_vals[k] - Xk[1]) - self.tg_min_forward*Xk[2]]
            lbg += [0.0]
            ubg += [inf]

        # Create an NLP solver
        prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
        opts = {'ipopt.print_level':0, 'print_time':0}
        solver = nlpsol('solver', 'ipopt', prob, opts);

        # Solve the NLP
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
        w_opt = sol['x'].full().flatten()

        x1_opt = w_opt[0::3]
        x2_opt = w_opt[1::3]
        u_opt = w_opt[2::3]

        return x1_opt,x2_opt,u_opt

    def AV_mpc_accel(self,s,v,ds_dt):

        # do prediciton:
        v_l_curr = v + ds_dt
        v_l_vals = np.ones(self.N+1,)*v_l_curr # predict continued speed into the future
        p_l_vals = np.cumsum(v_l_vals*self.dt) + s

        x_init = [v,0]

        _,_,u_opt = self.solve_cf_dms_smin(x_init,v_l_vals,p_l_vals)

        self.u_vals = np.array(u_opt)

        return u_opt[0]

    def get_accel(self):

        global lead_x
        global lead_rv
        global velocity

        print('lead_x: '+str(lead_x))
        print('lead_rv: '+str(lead_rv))
        print('velocity: '+str(velocity))



        return self.AV_mpc_accel(float(lead_x),float(velocity),float(lead_rv))


class mpc_forward_collision_avoider:
    def __init__(self):

        self.mpc_planner = MPC_min_forward_spacing_timegap(tg_min = 2.0,s_min = 15.0,v_des = 15.0, T = 5.0)

        rospy.init_node('mpctesting', anonymous=True)

        rospy.Subscriber(lead_x_topic, Float64, lead_x_callback)
        rospy.Subscriber(lead_rv_topic, Float64, lead_rv_callback)
        rospy.Subscriber(velocity_topic,Float64, velocity_callback)

        global mpc_cmd_accel_pub
        mpc_cmd_accel_pub = rospy.Publisher('/mpc_cmd_accel', Float64, queue_size=1000)
 
        self.rate = rospy.Rate(20)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                # Get accel cmd to publish:
                global mpc_cmd_accel_pub

                mpc_cmd_accel = self.mpc_planner.get_accel()

                mpc_cmd_accel_pub.publish(mpc_cmd_accel)


            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = mpc_forward_collision_avoider()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
