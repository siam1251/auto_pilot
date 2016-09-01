"""
A simple example of an animated plot
"""
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import std_msgs.msg as stdmsg
from nav_msgs.msg import Odometry
import sensor_msgs.msg as smsg
import geometry_msgs.msg as gmsg
from tf.transformations import euler_from_quaternion
from numpy.linalg import inv
import random
import math

#fig, ax = plt.subplots()
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=True)
ax.grid()

x = []*0       # x-array
y = []*0
line, = ax.plot(x,y,'r',lw=1, label="u1")
x2 = []
y2 = []
line2, = ax.plot(x2,y2,'g',lw=1, label="ekf")
#ax.axis("equal")
ax.set_ylim([5933500,5933837 + 200])
ax.set_xlim([332400,332650])
#ax.spines['left'].set_position('center')

odom_data = Odometry()
def odometry_update(data):
	print"s"
def animate(i):
	global odom_data
	global x, y
	#x.append(random.uniform(10, 20))
	#y.append(random.uniform(10, 20))
	# x.append(odom_data.pose.pose.position.x)
	# y.append(odom_data.pose.pose.position.y)
	# filter(lambda a: a != 0, x)
	# filter(lambda a: a != 0, y)
	# print odom_data.pose.pose.position.x
	# print odom_data.pose.pose.position.y
	
	# line.set_data(x,y)
	
	return line
def animate2(i):
	global line2
	return line2
#Init only required for blitting to give a clean slate.


def update_odom(data):
	global odom_data
	odom_data = data
	if odom_data.pose.pose.position.x != 0 and odom_data.pose.pose.position.x != 0:
		x.append(odom_data.pose.pose.position.x)
		y.append(odom_data.pose.pose.position.y)
	#print odom_data.pose.pose.position.x
	#print odom_data.pose.pose.position.y
	
	line.set_data(x,y)
	file1.write('%f %f\n' %(odom_data.pose.pose.position.x, odom_data.pose.pose.position.x))
	# global x, y
	# x.append(odom_data.pose.pose.position.x)
	# y.append(odom_data.pose.pose.position.y)
	
	# print odom_data.pose.pose.position.x
	# print odom_data.pose.pose.position.y
	# line.set_data(x,y)
	# print "s"
def update_control(data):
    global v
    tmp_x  = data.linear.x
    tmp_y  = data.linear.y
    v = math.sqrt(tmp_x*tmp_x + tmp_y*tmp_y)
def ekf_output(data):
	global x2, y2
	print data.x
	print data.y
	x2.append(data.x)
	y2.append(data.y)
	line2.set_data(x2, y2)
	file2.write('%f %f\n' %(data.x, data.y))
def subscribe():
	rospy.init_node('listener', anonymous=True)
	#rospy.Subscriber("/imu/data", smsg.Imu, update_imu)
	rospy.Subscriber("/odom", Odometry, update_odom) # gps data is converted to odometry 
	rospy.Subscriber("/husky_velocity_controller/cmd_vel", gmsg.Twist, update_control)
	rospy.Subscriber("/ekf", gmsg.Point, ekf_output)
	

if __name__ == '__main__':
	subscribe()
	file1 = open('/home/siam/catkin_ws/src/kalman/src/gps.txt', 'w')
	file2 = open('/home/siam/catkin_ws/src/kalman/src/gps_filtered.txt', 'w')
	simulation = animation.FuncAnimation(fig, animate, blit=False, frames=2000, interval=20, repeat=True)
	#simulation = animation.FuncAnimation(fig, animate2, blit=False, frames=2000, interval=20, repeat=True)
	#ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
	#    interval=25, blit=True)
	plt.show()
	#rospy.spin() 
