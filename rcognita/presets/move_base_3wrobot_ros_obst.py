#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Preset: a 3-wheel robot (kinematic model a. k. a. non-holonomic integrator) 
with connection to ROS.

"""

import os, sys
PARENT_DIR = os.path.abspath(__file__ + '/../..')
sys.path.insert(0, PARENT_DIR)
import rcognita

if os.path.abspath(rcognita.__file__ + "/../..") == PARENT_DIR:
    info = f"this script is being run using "+f"rcognita ({rcognita.__version__}) " + f"located in cloned repository at '{PARENT_DIR}'. " + f"If you are willing to use your locally installed rcognita, " + f"run this script ('{os.path.basename(__file__)}') outside " + f"'rcognita/presets'."
else:
    info = f"this script is being run using " + f"locally installed rcognita ({rcognita.__version__}). " + f"Make sure the versions match."
print("INFO:", info)

import pathlib
    
import warnings
import csv
from datetime import datetime
import threading
import os

from math import pi
import math
import time as time_lib

import numpy as np
from numpy import inf, matrix, cos, arctan2, sqrt, pi, sin, cos
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from rcognita import systems
from rcognita import controllers
from rcognita import loggers
from rcognita.utilities import on_key_press
from math_package.path_movement import PathMovement
from math_package.obstcl_parser import Obstacles_parser
import argparse

#------------------------------------imports for interaction with ROS

import rospy

from shapely.geometry import Point

from nav_msgs.msg import Odometry, Path
import tf.transformations as tftr
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import tf
#------------------------------------define ROS-preset class
class ROS_preset:

    def __init__(self, ctrl_mode, state_init, state_goal, my_ctrl_nominal, my_sys, my_ctrl_benchm, my_logger=None, datafiles=None):
        self.RATE = rospy.get_param('/rate', 10)
        self.lock = threading.Lock()
        self.odom_lock = threading.Lock()
        self.lidar_lock = threading.Lock()
        self.lidar_lock.acquire()
        # initialization
        self.state_init = state_init
        self.state_goal = state_goal
        self.system = my_sys

        self.ctrl_nominal = my_ctrl_nominal
        self.ctrl_benchm = my_ctrl_benchm

        self.dt = 0.0
        self.time_start = 0.0

        self.action_max = np.array([0.22, 2.84])
        self.cur_action_max = np.array([0.22, 2.84])

        self.constraints = []
        self.polygonal_constraints = []
        self.line_constrs = []
        self.circle_constrs = []
        self.ranges = []
        self.ranges_t0 = None
        self.ranges_t1 = None
        self.ranges_t2 = None
        self.counter = 0

        # connection to ROS topics
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=1)
        #self.sub_laser_scan = rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.sub_DWB_plan = rospy.Subscriber("/move_base_node/DWBLocalPlanner/local_plan", Path, self.DWB_callback, queue_size=10)
        self.sub_current_goal = rospy.Subscriber("move_base_node/current_goal", PoseStamped, self.current_goal_callback)
        self.state = np.zeros((3))
        self.dstate = np.zeros((3))
        self.new_state = np.zeros((3))
        self.new_dstate = np.zeros((3))
        self.listener = tf.TransformListener()  #  ======= to find distance to global goal base on odom pose

        self.datafiles = datafiles
        self.logger = my_logger
        self.ctrl_mode = ctrl_mode

        self.rotation_counter = 0
        self.prev_theta = 0
        self.new_theta = 0

        theta_goal = self.state_goal[2]

        self.rotation_matrix = np.array([
            [cos(theta_goal), -sin(theta_goal), 0],
            [sin(theta_goal),cos(theta_goal), 0],
            [0, 0, 1]
        ])
        self.odom_x = 1000
        self.odom_y = 1000
        self.obstacles_parser = Obstacles_parser(safe_margin_mult=1.35)
        self.flag_goal = False
        self.flag_path = False
    def odometry_callback(self, msg):
        # self.odom_lock.acquire()
        # Read current robot state
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
            self.odom_x = trans[0]
            self.odom_y = trans[1]

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        if self.flag_goal and self.flag_path:  #  ======= if there is a global goal the flag is True and controller is active
            current_rpy = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))
            theta = current_rpy[2]

            dx = msg.twist.twist.linear.x
            dy = msg.twist.twist.linear.y
            omega = msg.twist.twist.angular.z

            self.state = [x , y , theta]
            self.dstate = [dx, dy, omega]

            # Make transform matrix from `robot body` frame to `goal` frame
            local_goal = self.pose_des
            theta_goal = local_goal[2]

            rotation_matrix = np.array([
                [cos(theta_goal), -sin(theta_goal), 0],
                [sin(theta_goal),cos(theta_goal), 0],
                [0, 0, 1]
            ])
            self.rotation_matrix = rotation_matrix.copy()

            state_matrix = np.array([
                [local_goal[0]],
                [local_goal[1]],
                [0]
            ])

            t_matrix = np.block([
                [rotation_matrix, state_matrix],
                [np.array([[0, 0, 0, 1]])]
            ])

            inv_t_matrix = np.linalg.inv(t_matrix)

            if self.prev_theta * theta < 0 and abs(self.prev_theta - theta) > np.pi:
                if self.prev_theta < 0:
                    self.rotation_counter -= 1
                else:
                    self.rotation_counter += 1

            self.prev_theta = theta
            theta = theta + 2 * math.pi * self.rotation_counter

            new_theta = theta - theta_goal

            # POSITION transform
            temp_pos = [x, y, 0, 1]
            new_state = np.dot(inv_t_matrix, np.transpose(temp_pos))
            self.new_state = np.array([new_state[0], new_state[1], new_theta])
            # print("self.new_state", self.new_state)
            # print("self.state", self.state)
            # print("local_goal", local_goal)
            inv_R_matrix = inv_t_matrix[:3, :3]
            zeros_like_R = np.zeros(inv_R_matrix.shape)
            inv_R_matrix = np.linalg.inv(rotation_matrix)
            new_dstate = inv_R_matrix.dot(np.array([dx, dy, 0]).T)
            new_omega = omega
            self.new_dstate = [new_dstate[0], new_dstate[1], new_omega]

            cons = []
            for constr in self.constraints:
                cons.append(constr(self.new_state[:2]))
            f1 = np.max(cons) if len(cons) > 0 else 0

            if f1 > 0:
                print('COLLISION!!!')


            # self.lidar_lock.release()

    def DWB_callback(self, p):
        #  ======= DWB local planner updates the tempral goal for controller
        if len(p.poses) > 0:   #  ======= less than this number makes the controller unstable (very close local goal)

            self.x_DWBL = p.poses[-1 ].pose.position.x
            self.y_DWBL = p.poses[-1 ].pose.position.y
            self.t_DWBL = p.poses[-1 ].header.stamp.to_sec()
            self.orien_DWBL = p.poses[-1 ].pose.orientation
            self.z_DWBL = tf.transformations.euler_from_quaternion((self.orien_DWBL.x, self.orien_DWBL.y, self.orien_DWBL.z, self.orien_DWBL.w))[2]

            self.x_DWBL_0 = p.poses[0].pose.position.x
            self.y_DWBL_0 = p.poses[0].pose.position.y
            self.orien_DWBL_0 = p.poses[0].pose.orientation
            self.z_DWBL_0 = tf.transformations.euler_from_quaternion((self.orien_DWBL_0.x, self.orien_DWBL_0.y, self.orien_DWBL.z, self.orien_DWBL_0.w))[2]

            self.pose_des = [self.x_DWBL, self.y_DWBL, self.z_DWBL]
            self.pose_des_0 = [self.x_DWBL_0, self.y_DWBL_0, self.z_DWBL_0]

            self.flag_path = True       #  ======= gives signal to controller allowed to publish the velocity commands
        else:
            self.flag_path = False        #  ======= gives signal to the controller to stop publishing velocity commands


    def current_goal_callback(self, msg):
        goal_position = msg.pose.position
        goal_q = msg.pose.orientation
        goal_rpy = tf.transformations.euler_from_quaternion((goal_q.x, goal_q.y, goal_q.z, goal_q.w)) 

        self.state_goal = np.array([goal_position.x -  self.odom_x, goal_position.y -  self.odom_y, goal_rpy[2]])
        print("new goal destination",  self.state_goal)
        self.flag_goal = True

    def laser_scan_callback(self, dt):
        self.lidar_lock.acquire()
        # try:
        #     new_blocks, LL, CC, x, y = self.obstacles_parser.get_obstacles(np.array(dt.ranges), fillna='else', state=self.new_state)
        #     self.lines = LL
        #     self.circles = CC
        #     self.line_constrs = [self.obstacles_parser.get_buffer_area([[i[0].x, i[0].y], [i[1].x, i[1].y]], 0.178 * 1.75) for i in LL]

        #     self.circle_constrs = [Point(i.center[0], i.center[1]).buffer(i.r) for i in CC]
        #     self.constraints = self.obstacles_parser(np.array(dt.ranges), np.array(self.new_state))

        #     self.polygonal_constraints = self.line_constrs + self.circle_constrs
        #     #self.constraints = []

        # except ValueError as exc:
        #     print('Exception!', exc)
        #     self.constraints = []

        self.odom_lock.release()

    def spin(self, is_print_sim_step=False, is_log_data=True):
        rospy.loginfo('ROS-preset has been activated!')
        start_time = time_lib.time()
        rate = rospy.Rate(self.RATE)
        self.time_start = rospy.get_time()

        while not rospy.is_shutdown(): #and time_lib.time() - start_time < 180:
            t = rospy.get_time() - self.time_start
            self.t = t

            velocity = Twist()
            action = controllers.ctrl_selector(self.t, self.new_state, action_manual, self.ctrl_nominal, 
                                                self.ctrl_benchm, self.ctrl_mode, self.constraints, self.line_constrs)
            action = np.clip(action, -self.action_max, self.action_max)


            self.system.receive_action(action)
            self.ctrl_benchm.receive_sys_state(self.system._state)
            self.ctrl_benchm.upd_accum_obj(self.new_state, action)

            xCoord = self.new_state[0]
            yCoord = self.new_state[1]
            alpha = self.new_state[2]

            stage_obj = self.ctrl_benchm.stage_obj(self.new_state, action)
            accum_obj = self.ctrl_benchm.accum_obj_val

            self.ctrl_benchm.receive_sys_state(self.new_state)
            

            velocity.linear.x = action[0]
            velocity.angular.z = action[1]


            if (np.sqrt((self.state_goal[0] - self.state[0])**2 + (self.state_goal[1] - self.state[1])**2) < 0.3
                and (np.abs(np.degrees(self.state_goal[2] - self.state[2])) % 360 < 15) and t > 10):
            #          (345 < np.abs(np.degrees(alpha - self.state_init[2])) % 360 < 360))) and t > 10
                print('FINAL RESULTS!!!')
                print(t, xCoord, yCoord, alpha, stage_obj, accum_obj, action)
                velocity.linear.x = 0
                velocity.angular.z = 0
                break
  

            self.pub_cmd_vel.publish(velocity)
            rate.sleep()

        velocity = Twist()
        velocity.linear.x = 0.
        velocity.angular.z = 0.
        self.pub_cmd_vel.publish(velocity)

        rospy.loginfo('ROS-preset has finished working')


if __name__ == "__main__":
    rospy.init_node('ROS_preset_node')

    #----------------------------------------Set up dimensions
    dim_state = 3
    dim_input = 2
    dim_output = dim_state
    dim_disturb = 2

    dim_R1 = dim_output + dim_input
    dim_R2 = dim_R1

    description = "Agent-environment preset: a 3-wheel robot (kinematic model a. k. a. non-holonomic integrator)."

    parser = argparse.ArgumentParser(description=description)

    parser.add_argument('--ctrl_mode', metavar='ctrl_mode', type=str,
                        choices=['manual',
                                'nominal',
                                'MPC',
                                'RQL',
                                'SQL',
                                'JACS'],
                        default='nominal',
                        help='Control mode. Currently available: ' +
                        '----manual: manual constant control specified by action_manual; ' +
                        '----nominal: nominal controller, usually used to benchmark optimal controllers;' +                     
                        '----MPC:model-predictive control; ' +
                        '----RQL: Q-learning actor-critic with Nactor-1 roll-outs of stage objective; ' +
                        '----SQL: stacked Q-learning; ' + 
                        '----JACS: joint actor-critic (stabilizing), system-specific, needs proper setup.')
    parser.add_argument('--dt', type=float, metavar='dt',
                        default=0.05,
                        help='Controller sampling time.' )
    parser.add_argument('--t1', type=float, metavar='t1',
                        default=150.0,
                        help='Final time of episode.' )
    parser.add_argument('--state_init', type=str, nargs="+", metavar='state_init',
                        default=['2', '2', 'pi'],
                        help='Initial state (as sequence of numbers); ' + 
                        'dimension is environment-specific!')
    parser.add_argument('--is_log_data', type=bool,
                        default=True,
                        help='Flag to log data into a data file. Data are stored in simdata folder.')
    parser.add_argument('--is_visualization', type=bool,
                        default=True,
                        help='Flag to produce graphical output.')
    parser.add_argument('--is_print_sim_step', type=bool,
                        default=True,
                        help='Flag to print simulation data into terminal.')
    parser.add_argument('--is_est_model', type=bool,
                        default=False,
                        help='Flag to estimate environment model.')
    parser.add_argument('--model_est_stage', type=float,
                        default=2.0,
                        help='Seconds to learn model until benchmarking controller kicks in.')
    parser.add_argument('--model_est_period_multiplier', type=float,
                        default=1,
                        help='Model is updated every model_est_period_multiplier times dt seconds.')
    parser.add_argument('--model_order', type=int,
                        default=5,
                        help='Order of state-space estimation model.')
    parser.add_argument('--prob_noise_pow', type=float,
                        default=8,
                        help='Power of probing (exploration) noise.')
    parser.add_argument('--action_manual', type=float,
                        default=[0.22, 0.0], nargs='+',
                        help='Manual control action to be fed constant, system-specific!')
    parser.add_argument('--Nactor', type=int,
                        default=4,
                        help='Horizon length (in steps) for predictive controllers.')
    parser.add_argument('--pred_step_size_multiplier', type=float,
                        default=2.0,
                        help='Size of each prediction step in seconds is a pred_step_size_multiplier multiple of controller sampling time dt.')
    parser.add_argument('--buffer_size', type=int,
                        default=200,
                        help='Size of the buffer (experience replay) for model estimation, agent learning etc.')
    parser.add_argument('--stage_obj_struct', type=str,
                        default='quadratic',
                        choices=['quadratic',
                                'biquadratic'],
                        help='Structure of stage objective function.')
    parser.add_argument('--R1_diag', type=float, nargs='+',
                        default=[10, 10, 1, 0, 0],
                        help='Parameter of stage objective function. Must have proper dimension. ' +
                        'Say, if chi = [observation, action], then a quadratic stage objective reads chi.T diag(R1) chi, where diag() is transformation of a vector to a diagonal matrix.')
    parser.add_argument('--R2_diag', type=float, nargs='+',
                        default=[[10, 2, 1, 0, 0], [0, 10, 2, 0, 0], [0, 0, 10, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]],
                        help='Parameter of stage objective function . Must have proper dimension. ' + 
                        'Say, if chi = [observation, action], then a bi-quadratic stage objective reads chi**2.T diag(R2) chi**2 + chi.T diag(R1) chi, ' +
                        'where diag() is transformation of a vector to a diagonal matrix.')
    parser.add_argument('--Ncritic', type=int,
                        default=50,
                        help='Critic stack size (number of temporal difference terms in critic cost).')
    parser.add_argument('--gamma', type=float,
                        default=1.0,
                        help='Discount factor.')
    parser.add_argument('--critic_period_multiplier', type=float,
                        default=5.0,
                        help='Critic is updated every critic_period_multiplier times dt seconds.')
    parser.add_argument('--critic_struct', type=str,
                        default='quad-nomix', choices=['quad-lin',
                                                    'quadratic',
                                                    'quad-nomix',
                                                    'quad-mix'],
                        help='Feature structure (critic). Currently available: ' +
                        '----quad-lin: quadratic-linear; ' +
                        '----quadratic: quadratic; ' +
                        '----quad-nomix: quadratic, no mixed terms; ' +
                        '----quad-mix: quadratic, mixed observation-action terms (for, say, Q or advantage function approximations).')
    parser.add_argument('--actor_struct', type=str,
                        default='quad-nomix', choices=['quad-lin',
                                                    'quadratic',
                                                    'quad-nomix'],
                        help='Feature structure (actor). Currently available: ' +
                        '----quad-lin: quadratic-linear; ' +
                        '----quadratic: quadratic; ' +
                        '----quad-nomix: quadratic, no mixed terms.')

    args = parser.parse_args()

    #----------------------------------------Post-processing of arguments
    # Convert `pi` to a number pi
    for k in range(len(args.state_init)):
        args.state_init[k] = eval( args.state_init[k].replace('pi', str(np.pi)) )

    args.state_init = np.array(args.state_init)
    args.action_manual = np.array(args.action_manual)

    pred_step_size = args.dt * args.pred_step_size_multiplier
    model_est_period = args.dt * args.model_est_period_multiplier
    critic_period = args.dt * args.critic_period_multiplier

    R1 = np.diag(np.array(args.R1_diag))
    R2 = np.diag(np.array(args.R2_diag))

    assert args.t1 > args.dt > 0.0
    assert args.state_init.size == dim_state

    globals().update(vars(args))

    #----------------------------------------(So far) fixed settings
    is_disturb = 0

    # Disturbance
    sigma_q = 1e-3 * np.ones(dim_disturb)
    mu_q = np.zeros(dim_disturb)
    tau_q = np.ones(dim_disturb)

    is_dyn_ctrl = 0

    x0 = np.zeros(dim_state)

    t0 = 0
    Nruns = 1

    x0 = np.zeros(dim_state)
    action_init = np.zeros(dim_input)
    q0 = np.zeros(dim_disturb)

    # Solver
    atol = 1e-5
    rtol = 1e-3

    # xy-plane
    xMin = -10
    xMax = 10
    yMin = -10
    yMax = 10

    # Model estimator stores models in a stack and recall the best of model_est_checks
    model_est_checks = 0

    # Control constraints
    v_min = -0.22
    v_max = 0.22
    omega_min = -2.84
    omega_max = 2.84
    ctrl_bnds=np.array([[v_min, v_max], [omega_min, omega_max]])

    state_goal = state_init.copy()

    #----------------------------------------Initialization : : system
    my_sys = systems.Sys3WRobotNI(sys_type="diff_eqn",
                                        dim_state=dim_state,
                                        dim_input=dim_input,
                                        dim_output=dim_output,
                                        dim_disturb=dim_disturb,
                                        pars=[],
                                        ctrl_bnds=ctrl_bnds,
                                        is_dyn_ctrl=is_dyn_ctrl,
                                        is_disturb=is_disturb,
                                        pars_disturb=[sigma_q, mu_q, tau_q])

    observation_init = my_sys.out(state_init)

    #----------------------------------------Initialization : : model

    #----------------------------------------Initialization : : controller
    my_ctrl_nominal = controllers.CtrlNominal3WRobotNI(ctrl_gain=0.02, ctrl_bnds=ctrl_bnds, t0=t0, sampling_time=dt)

    # Predictive optimal controller
    my_ctrl_opt_pred = controllers.CtrlOptPred(dim_input,
                                            dim_output,
                                            ctrl_mode,
                                            ctrl_bnds = ctrl_bnds,
                                            action_init = [],
                                            t0 = t0,
                                            sampling_time = dt,
                                            Nactor = Nactor,
                                            pred_step_size = pred_step_size,
                                            sys_rhs = my_sys._state_dyn,
                                            sys_out = my_sys.out,
                                            state_sys = x0,
                                            prob_noise_pow = prob_noise_pow,
                                            is_est_model = is_est_model,
                                            model_est_stage = model_est_stage,
                                            model_est_period = model_est_period,
                                            buffer_size = buffer_size,
                                            model_order = model_order,
                                            model_est_checks = model_est_checks,
                                            gamma = gamma,
                                            Ncritic = Ncritic,
                                            critic_period = critic_period,
                                            critic_struct = critic_struct,
                                            stage_obj_struct = stage_obj_struct,
                                            stage_obj_pars = [R1],
                                            observation_target = [])

    # Stabilizing RL agent
    my_ctrl_RL_stab = controllers.CtrlRLStab(dim_input,
                                            dim_output,
                                            ctrl_mode,
                                            ctrl_bnds = ctrl_bnds,
                                            action_init = action_init,
                                            t0 = t0,
                                            sampling_time = dt,
                                            Nactor = Nactor,
                                            pred_step_size = pred_step_size,
                                            sys_rhs = my_sys._state_dyn,
                                            sys_out = my_sys.out,
                                            state_sys = state_init,
                                            prob_noise_pow = prob_noise_pow,
                                            is_est_model = is_est_model,
                                            model_est_stage = model_est_stage,
                                            model_est_period = model_est_period,
                                            buffer_size = buffer_size,
                                            model_order = model_order,
                                            model_est_checks = model_est_checks,
                                            gamma = gamma,
                                            Ncritic = Ncritic,
                                            critic_period = critic_period,
                                            critic_struct = critic_struct,
                                            actor_struct = actor_struct,
                                            stage_obj_struct = stage_obj_struct,
                                            stage_obj_pars = [R1],
                                            observation_target = [],
                                            safe_ctrl = my_ctrl_nominal,
                                            safe_decay_rate = 1e-4)

    if ctrl_mode == 'JACS':
        my_ctrl_benchm = my_ctrl_RL_stab
    else:
        my_ctrl_benchm = my_ctrl_opt_pred

    #----------------------------------------Initialization : : logger
    if os.path.basename( os.path.normpath( os.path.abspath(os.getcwd()) ) ) == 'presets':
        data_folder = '../simdata'
    else:
        data_folder = 'simdata'

    pathlib.Path(data_folder).mkdir(parents=True, exist_ok=True) 

    date = datetime.now().strftime("%Y-%m-%d")
    time = datetime.now().strftime("%H:%M:%S")
    datafiles = [None] * Nruns

    for k in range(0, Nruns):
        datafiles[k] = data_folder + '/' + 'ROS__' + my_sys.name + '__' + ctrl_mode + '__' + date + '__' + time + '__run{run:02d}.csv'.format(run=k+1)
        
        if is_log_data:
            print('Logging data to:    ' + datafiles[k])
                
            with open(datafiles[k], 'w', newline='') as outfile:
                writer = csv.writer(outfile)
                writer.writerow(['System', my_sys.name ] )
                writer.writerow(['Controller', ctrl_mode ] )
                writer.writerow(['dt', str(dt) ] )
                writer.writerow(['state_init', str(state_init) ] )
                writer.writerow(['is_est_model', str(is_est_model) ] )
                writer.writerow(['model_est_stage', str(model_est_stage) ] )
                writer.writerow(['model_est_period_multiplier', str(model_est_period_multiplier) ] )
                writer.writerow(['model_order', str(model_order) ] )
                writer.writerow(['prob_noise_pow', str(prob_noise_pow) ] )
                writer.writerow(['Nactor', str(Nactor) ] )
                writer.writerow(['pred_step_size_multiplier', str(pred_step_size_multiplier) ] )
                writer.writerow(['buffer_size', str(buffer_size) ] )
                writer.writerow(['stage_obj_struct', str(stage_obj_struct) ] )
                writer.writerow(['R1_diag', str(R1_diag) ] )
                writer.writerow(['R2_diag', str(R2_diag) ] )
                writer.writerow(['Ncritic', str(Ncritic) ] )
                writer.writerow(['gamma', str(gamma) ] )
                writer.writerow(['critic_period_multiplier', str(critic_period_multiplier) ] )
                writer.writerow(['critic_struct', str(critic_struct) ] )
                writer.writerow(['actor_struct', str(actor_struct) ] )   
                writer.writerow(['t [s]', 'x [m]', 'y [m]', 'alpha [rad]', 'stage_obj', 'accum_obj', 'v [m/s]', 'omega [rad/s]'] )

    # Do not display annoying warnings when print is on
    if is_print_sim_step:
        warnings.filterwarnings('ignore')
        
    my_logger = loggers.Logger3WRobotNI()

    ros_preset_task = ROS_preset(ctrl_mode, [0, 0, 0], state_goal, my_ctrl_nominal, my_sys, my_ctrl_benchm, my_logger, datafiles)
    
    ros_preset_task.spin(is_print_sim_step=is_print_sim_step, is_log_data=is_log_data)