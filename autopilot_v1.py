#!/usr/bin/env python
#PKG = 'mrobot_simple_nav'
#import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
#roslib.load_manifest('mrobot_simple_nav_msgs')

from dynamic_systems import *
from time import *
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

import rospy
import numpy as np

from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
#from mrobot_simple_nav_msgs.msg import sys_states_all
theta = 0
state0 = 0
state1 = 0
state2 = 0
state3 = 0
state4 = 0
state5 = 0
state6 = 0
state7 = 0
state8 = 0
state9 = 0

def read_data(file_name):
    with open(file_name) as f:
        data = f.read()
    data = data.strip().split('\n')
    #print data
    print np.shape(data)
    x = [row.strip().split(' ') for row in data]
    #print x
    #x = np.
    
    #print y
    #y = [row.split(' ')[1] for row in data]
    #d = [x,y]
    # x = [[foo for i in range(10)] for j in range(10)]
    intlist = [[float(k) for k in y] for y in x]
    print intlist
    return intlist

pose_robot = PoseWithCovarianceStamped()
pose_init_flag = 0
# ----------------------------------------------
'''def update_states(data):
    global state0
    global state1
    global state2
    global state3
    global state4
    global state5
    global state6
    global state7
    global state8
    global state9

    state0 = data.state0
    state1 = data.state1
    state2 = data.state2
    state3 = data.state3
    state4 = data.state4
    state5 = data.state5
    state6 = data.state6
    state7 = data.state7
    state8 = data.state8
    state9 = data.state9

def update_EKF(data):    
    global pose_robot
    global pose_init_flag
    pose_init_flag = 1;
    pose_robot = data;
'''

def update_SIM(data):    
    global pose_robot
    global pose_init_flag
    pose_init_flag = 1;
    pose_robot = data;
    print data
def update_ODO(data):    
    global pose_robot
    global pose_init_flag
    pose_init_flag = 1;
    pose_robot = data;
    print data
def update_theta(msg):
	global theta
	euler = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
	theta = euler[2]+2.5
#-----------------------------------------

def main_loop():
    way_points = read_data('/home/siam/catkin_ws/src/getMap/trajectories/input.txt')
    print way_points
    global state0
    global state1
    global state2
    global state3
    global state4
    global state5
    global state6
    global state7
    global state8
    global state9

    global pose_robot
    global theta

    #rospy.Subscriber('/sys_states/sys_states_all',sys_states_all,update_states)
    # rospy.Subscriber('/ekf', PoseWithCovariance, update_EKF)
    #rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, update_SIM)
    rospy.Subscriber('/odom', Odometry, update_ODO)
    rospy.Subscriber('/imu/data', Imu, update_theta)


    # Maybe publish a state so that when it reaches a WP it tells something, or the state can be about what it is doing.
    # pub = rospy.Publisher('/sys_states/state2',UInt16)

    pub_vel = rospy.Publisher('/cmd_vel',Twist)

    rospy.init_node('autopilot', anonymous=True)

    r = rospy.Rate(50)
    state = UInt16() # to be used later


    cmd = Twist()

    dt = 0.02

    Kp = 0.5
    Kd = 0.1
    Ki = 0.1

    th_pid     = PID_controller(Kp, Ki, Kd, deadband = 0., u_min = -1., u_max = 1., e_int_min = -0.2, e_int_max = 0.2, dt = dt)
    
    Kp = 0.5
    Kd = 0.0
    Ki = 0.1

    xy_pid     = PID_controller(Kp, Ki, Kd, deadband = 0., u_min = -1., u_max = 1, e_int_min = -0.1, e_int_max = 0.1, dt = dt)
    #320986.689196 5531097.099088
    #320983.524263 5531110.395138
    #320980.545503 5531124.881879
    #global theta
    #way_points = [ [332607.99606, 5933837.34219], [332546.337235, 5933838.56433], [332549.351423, 5933864.76626], [332587.268054, 5933864.13036] ]
    #way_points = [[320986.689196, 5531097.099088],[320983.524263, 5531110.395138],[320980.545503, 5531124.881879]]
    
    
    way_point_index = 0
    way_points_end_flag = 0

    rx = way_points[way_point_index][0]
    ry = way_points[way_point_index][1]
    print rx
    print ry
    error_th    = [0.0, 0.0, 0.0]
    error_dist  = [0.0, 0.0, 0.0]

    u = [0,0]


    while pose_init_flag == 0:
        print "No pose yet"
        sleep(2.0)
      
    while not rospy.is_shutdown():


        if not way_points_end_flag:
            px  = pose_robot.pose.pose.position.x
            py  = pose_robot.pose.pose.position.y
            pth = theta

            p_err = ((rx-px)**2 +(ry-py)**2)**0.5

            if p_err < 5.0:
                way_point_index += 1
                if way_point_index < len(way_points):
                    rx = way_points[way_point_index][0]
                    ry = way_points[way_point_index][1]
                    p_err = ((rx-px)**2 +(ry-py)**2)**0.5
                else:
                    p_err = 0
                    way_points_end_flag = 1


            rth = math.atan2((ry-py),(rx-px))
            print('distance error %f /n' %(p_err));
            print('theta error %f /n' %(rth));
            th_err = math.atan2(sin(rth-pth),cos(rth-pth))

            w = th_pid.calc_output(-th_err, dt)
            v = xy_pid.calc_output(-p_err, dt)
            cmd.linear.x = v
            cmd.angular.z = w
            print('%f %f' %(w, v))

        else:
            break
            '''w = 0.0
            v = 0.0

        if state0 == 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        if state0 == 1:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        if state0 == 2:

            cmd.linear.x = v
            cmd.angular.z = w'''


        # To be used later, not sure what number of state
        # if not(state2 == state):
        #     pub.publish(state) 

        pub_vel.publish(cmd)

        r.sleep()

if __name__ == "__main__": 
    main_loop()












