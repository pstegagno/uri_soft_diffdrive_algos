#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import re
import random as r


linvel = 0.1
mindist = 0.6
noise = 0




# this is a global variable: all functions and methods in this file have access to it
cmd_pub = rospy.Publisher("noisy_odom", Odometry , queue_size=1)


# this is our callback function: it is executed any time a message on the specified topic
# is received. In our case, the specified topic is /drone/gt_pose.
# The message received will be available in the function through the variable msg  
def AddNoise(msg):
		
    # to use a global variable in a function, you must explicitly
    # redefine with the global attribute
    global cmd_pub
    global linvel
    global pos_est_x
    global pos_est_y
    global pos_est_th
    global lin_noise
    
    dt = .12
    
    eps_lin = np.random.normal(0,lin_noise)
    #eps_y = np.random.normal(0,lin_noise)
    eps_th = np.random.normal(0,ang_noise)

    lin_vel = msg.linear.x
    ang_vel = msg.angular.z
    
    #Qx = msg.pose.pose.orientation.x
    #Qy = msg.pose.pose.orientation.y
    #Qz = msg.pose.pose.orientation.z
    #Qw = msg.pose.pose.orientation.w
    
    noisy_lin_vel = lin_vel + eps_lin
    noisy_ang_vel = ang_vel + eps_th
    
    
    pos_est_th_new = pos_est_th + (noisy_ang_vel)*dt
    pos_est_x = pos_est_x + dt*noisy_lin_vel*math.cos((pos_est_th_new+pos_est_th)/2)
    pos_est_y = pos_est_y + dt*noisy_lin_vel*math.sin((pos_est_th_new+pos_est_th)/2)
    
    #pos_est_x = pos_est_x + dt*(lin_vel+eps_lin)*math.cos((pos_est_th)
    #pos_est_y = pos_est_y + dt*(lin_vel+eps_lin)*math.sin(pos_est_th)
    
    pos_est_th = pos_est_th_new
    
    #t3 = 2.0 * (Qw * Qz + Qx * Qy)
    #t4 = 1.0 -2.0 * (Qy*Qy + Qz*Qz)
    #yaw_z = math.atan2(t3,t4)
    
   
    roll = 0
    pitch = 0
    yaw = pos_est_th
    
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(pos_est_th/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(pos_est_th/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(pos_est_th/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(pos_est_th/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(pos_est_th/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(pos_est_th/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(pos_est_th/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(pos_est_th/2)
 
       
    state_est = Odometry()
    state_est.pose.pose.position.x = pos_est_x
    state_est.pose.pose.position.y = pos_est_y
    state_est.pose.pose.orientation.x = qx
    state_est.pose.pose.orientation.y = qy
    state_est.pose.pose.orientation.z = qz
    state_est.pose.pose.orientation.w = qw
    state_est.twist.twist.linear.x = noisy_lin_vel
    state_est.twist.twist.angular.z = noisy_ang_vel
    
    cmd_pub.publish(state_est)
    




def odom_integrator():

    global linvel
    global mindist
    global lin_noise
    global ang_noise
    global pos_est_x
    global pos_est_y
    global pos_est_th

    rospy.init_node('noise', anonymous=False)
    

    
    nodename = rospy.get_name()
    linvel = rospy.get_param(nodename+"/linvel", 0.1);
    mindist = rospy.get_param(nodename+"/mindist", 0.6);
    lin_noise = rospy.get_param(nodename+"/lin_noise", 0.02);
    ang_noise = rospy.get_param(nodename+"/ang_noise", 0.005);
       
    
    #rospy.Subscriber("odom", Odometry , AddNoise) #deleted queue_size=1
    rospy.Subscriber("cmd_vel", Twist , AddNoise)
    
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    global pos_est_x
    global pos_est_y
    global pos_est_th

    nodename = rospy.get_namespace()
    team = rospy.get_param(nodename+"odometry/team_size");
    IPs = '/home/k-2so/catkin_ws/src/uri_soft_ah/ajh_swarm_slam/init_pose_'+str(int(team))+'.csv'
    init = np.loadtxt(IPs,delimiter=' ', dtype=str)
    my_num = re.findall(r'\d+', nodename)
    r = int(my_num[0])-1
    
    pos_est_x = float(init[r,0])
    pos_est_y = float(init[r,1])
    pos_est_th = 0

    odom_integrator()



