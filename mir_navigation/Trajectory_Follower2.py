#!/usr/bin/env python3

import rospy
import threading
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from numpy import matrix, cos, arctan2, sqrt, pi, sin, cos, sign
from math import atan2
import random
import tf.transformations as tftr

class TrajectoryPlotter:
    def __init__(self):
    	self.lock = threading.Lock()
    	#rospy.init_node('trajectory_plotter')
    	self.path_sub = rospy.Subscriber('/move_base_node/DWBLocalPlanner/local_plan', Path, self.path_callback)
    	self.path_sub2 = rospy.Subscriber('/move_base_node/SBPLLatticePlanner/plan', Path, self.Globpath_callback)
    	self.sub_odom = rospy.Subscriber("/odom", Odometry, self.Velocity_publisher)
    	#self.path_sub3 = rospy.Subscriber('/move_base_node/DWBLocalPlanner/transformed_global_plan',
    	#Path,self.TFGlobpath_callback)
    	self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
    	
    	self.fig, self.ax = plt.subplots()
    	self.x_data = []
    	self.y_data = []
    	self.globx_data = []
    	self.globy_data = []
    	self.end = False
    	self.local_goal_x=[]

    def path_callback(self, path_msg):
        poses = path_msg.poses
        self.x_data = []
        self.y_data = []
        self.timestamps = []
        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            timestamp = pose.header.stamp.to_sec()
            self.x_data.append(x)
            self.y_data.append(y)
            self.local_goal_x=self.x_data[-1]
            self.timestamps.append(timestamp)
        print('LocalLocalLocalLocalLocalLocalLocalLocalLocal')
        print('LocalLocalLocalLocalLocalLocalLocalLocalLocal')
        print('Received new  local trajectory:')
        print('X:', self.x_data)
        print('Y:', self.y_data)
        print('Local goal X:', self.local_goal_x)
        print('Timestamps:', self.timestamps)
        print('Final global goal x is=',self.globx_data[-1])
        print('Final global goal  y is=',self.globy_data[-1])
        
    def Globpath_callback(self, globpath_msg):
        globposes = globpath_msg.poses
        self.globx_data = []
        self.globy_data = []
        dummy_x=[]
        dummy_y=[]
        for pose in globposes:
            globx = pose.pose.position.x
            globy = pose.pose.position.y
            
            dummy_x.append(globx)
            dummy_y.append(globy)
        #self.globx_data=dummy_x-dummy_x[0] 
        #self.globy_data=dummy_y-dummy_y[0]  
        self.globx_data = [xx - 10 for xx in dummy_x]  #dummy_x[0]
        self.globy_data = [yy - 10 for yy in dummy_y] #dummy_y[0]
        print('GlobalGlobalGlobalGlobalGlobalGlobalGlobalGlobal')
        print('GlobalGlobalGlobalGlobalGlobalGlobalGlobalGlobal')
        print('Received new global trajectory:')
        print('Global_X:', self.globx_data)
        print('Global_Y:', self.globy_data)
        
    def Velocity_publisher(self, msg):
    	print('vel pub started!!!')
    	self.lock.acquire()
    	# read current robot state
    	cur_position = msg.pose.pose.position
    	cur_q = msg.pose.pose.orientation
    	cur_rpy = tftr.euler_from_quaternion((cur_q.x, cur_q.y, cur_q.z, cur_q.w))  # roll pitch yaw
    	cur_rot_z = cur_rpy[2]
    	self.lock.release()
    	
    	
    	# ******************************************
    	# calculating min distance between current position and the points on local trajectory
    	x=self.x_data
    	y=self.y_data
    	distances = []
    	for i in range(len(x)):
    		distance = sqrt((cur_position.x-x[i])**2 + (cur_position.y-y[i])**2)
    		distances.append(distance)
    	min_distance = min(distances)
    	min_index = distances.index(min_distance)
    	if min_index> len(x)-3:
    		print('!!! min distance point was put in the point(last-2) !!!')
    		min_index=len(x)-3
    		
    	#print('point_min_dis_2mir=',min_index)
    	# ******************************************
    	
    	
    	# ******************************************
    	# calculating the trajectory slop @ the point with min distance to MIR and 
    	# the diff between this slope and current robot angular pos.
    	traj_slop=atan2((y[min_index+1]-y[min_index]),(x[min_index+1]-x[min_index]+1e-6))
    	
    	err_ang=traj_slop-cur_rot_z
    	if err_ang>pi/2:
    		err_ang+=-2*pi
    	elif err_ang<-pi/2:
    		err_ang+=+2*pi
    	# ******************************************
    	
    	
    	
    	# ******************************************
    	# calculating the lateral error from the robot to the local trajectory and its sign (but the result
    	# is not used in this version)
    	v_mir=(cos(cur_rot_z),sin(cur_rot_z)) # a unit vector based on MIR yaw angle
    		
    	if x[min_index+1]==x[min_index]:
    		lat_err=abs(cur_position.x-x[min_index])
    		x_int=x[min_index]
    		y_int=cur_position.y
    		
    	else:
    		m=(y[min_index+1]-y[min_index])/(x[min_index+1]-x[min_index])
    		h=y[min_index]-m*x[min_index]
    		lat_err=abs(cur_position.y-m*cur_position.x-h)/sqrt(1+m**2)
    		# determining the direction of lat err
    		hp=cur_position.y+1/m*cur_position.x
    		y_int=(h+hp*m**2)/(1+m**2)
    		x_int=(y_int-h)/m
    		
    	v_lat=(x_int-cur_position.x , y_int-cur_position.y)
    	cross_prod=v_mir[0]*v_lat[1]-v_mir[1]*v_lat[0]
    	sgn=sign(cross_prod)
    	# ******************************************
    	
    	# ******************************************
    	# calculating how the trajectory turns in the next few points (after min distance point form MIR), and calculate the robot 
    	# yaw rate accordingly in the next sections
    	
    	teta1=traj_slop#atan2((y[min_index+1]-y[min_index]),(x[min_index+1]-x[min_index]+1e-6))
    	
    	teta2=atan2((y[min_index+2]-y[min_index+1]),(x[min_index+2]-x[min_index+1]+1e-6))
    	
    	d1=sqrt((y[min_index+1]-y[min_index])**2+(x[min_index+1]-x[min_index])**2)
    	d2=sqrt((y[min_index+2]-y[min_index+1])**2+(x[min_index+2]-x[min_index+1])**2)
    	
    	
    	del_teta=teta2-teta1
    	if del_teta>pi/2:
    		del_teta+=-2*pi
    	elif del_teta<-pi/2:
    		del_teta+=+2*pi 
    		
    	if (abs(del_teta)>pi/4) or (abs(err_ang)>pi/4):
    		print('**********Hey!!!! There should be an ERROR with angles in local trajectory!!!!!!!*****************')
    	# ******************************************



    	velocity = Twist()
    	
    	
    	
    	
    	# ******************************************
    	# Publishing velocities
    	
    	# Calculating the current distance to goal in order to stop MIR in the vicinity of the goal
    	
    	d2goal= sqrt((cur_position.y-self.globy_data[-1])**2+(cur_position.x-self.globx_data[-1])**2)
    	
    	
    	if d2goal<.3 or (x[min_index+1]==x[min_index] and y[min_index+1]==y[min_index]):
    		long_vel=0.0*d2goal
    		ang_vel_coeff=0
    		yawrate_angle_coeff=0
    		yawrate_lat_coeff=0
    		yawrate_angle=0
    		yawrate_lat=0
    		print('Approximately last point achieved or an error occured')
    	#elif d2goal<.15 or (x[min_index+1]==x[min_index] and y[min_index+1]==y[min_index]):
    		#long_vel=0.0*d2goal
    		#ang_vel_coeff=0
    		#yawrate_angle_coeff=0
    		#yawrate_lat_coeff=0
    		#yawrate_angle=0
    		#yawrate_lat=0
    	else:
    		long_vel=0.5
    		ang_vel_coeff=2
    		yawrate_angle_coeff=0.2
    		yawrate_lat_coeff=0
    		yawrate_angle=err_ang
    		yawrate_lat=1.0*sgn*lat_err
    		
    		
    	t_tot=(d1+d2)/long_vel  # total time to go by long_vel m/s from the nearest point (p1) to p2 and then p3
    	ang_vel=del_teta/(t_tot+1e-6)
    	
    	velocity.linear.x =long_vel
    	velocity.angular.z=ang_vel_coeff*ang_vel + yawrate_angle_coeff*yawrate_angle + yawrate_lat_coeff*yawrate_lat  #20*(del_teta)
    	
    	
    	self.pub_cmd_vel.publish(velocity)
    	
    	
    	# ******************************************
    	# Print miscellaneous data 
    	
    	show_err_ang=err_ang*180/pi
    	show_traj_slop=traj_slop*180/pi
    	show_yaw_ang=cur_rot_z*180/pi
    	
    	print('********miscellaneous data********')
    	print('x_MIR=',cur_position.x, 'y_MIR=', cur_position.y)
    	print( 'total time is=', t_tot)
    	print('yaw rate is', velocity.angular.z, 'yawrate_angle is',yawrate_angle, 'yawrate_lat is', yawrate_lat , 'delta_teta', del_teta)
    	print('err_ang=', show_err_ang, 'show_traj_slop=',show_traj_slop, 'show_yaw_ang=',show_yaw_ang)
    	
    	
    	
    	
    	
    
    
    
           
# ********************************************************
    #def update_plot(self):
        
        #colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k'] # predefined color order
        #color = random.choice(colors) # randomly choose a color from the order
        #self.ax.plot(self.x_data, self.y_data, color+'-')
        #self.ax.plot(self.globx_data, self.globy_data, 'b--', linewidth=.5)
        #self.ax.set_xlim([-10, 10])
        #self.ax.set_ylim([-10, 10])
        #self.ax.set_xlabel('X')
        #self.ax.set_ylabel('Y')
        #self.ax.set_title('Robot Trajectory')
        #plt.draw()
        #plt.pause(0.001)
# ********************************************************
    #def run(self):
        #rate = rospy.Rate(30)  # 10 Hz
        #while not rospy.is_shutdown():
            ##self.update_plot()
            ##self.Velocity_publisher()
            #rate.sleep()
    def spin(self):
        rospy.loginfo('Task started!')
        rate = rospy.Rate(3)
        self.end = False
        while not rospy.is_shutdown():
        	rate.sleep()
        	#self.Velocity_publisher()
        rospy.loginfo('Task completed!')


if __name__ == '__main__':
    #try:
        #tp = TrajectoryPlotter()
        #tp.run()
    #except rospy.ROSInterruptException:
        #pass
    rospy.init_node('trajectoryprinter_node')
    tp = TrajectoryPlotter()
    tp.spin()

