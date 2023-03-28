#!/usr/bin/env python3

from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time

import sys
import os

print("Python version")
print (sys.version)

print("running pure pursuit")

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Clock
from  tf.transformations import euler_from_quaternion
# from scipy.spatial.transform import Rotation as R
import quaternion

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation

import utils_pp
import utils_viz
import tf

# cmd = "rosnode kill /ekf_localization"
# os.system(cmd)


# ================================================
# SET USER PARAMETERS
# ================================================
num = 8
x_world = 10
y_world = 10

x_fig = 10
y_fig = 10 


class pure_pursuit_node_class:
	def __init__(self):
		self.sub_clock = rospy.Subscriber('/clock', Clock, self.callback_clock)
		self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)
		# self.sub_gazebo_link = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback_gazebo_link_states)

		self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.pub_curvature = rospy.Publisher('curvature', Float32, queue_size = 1)
		# self.pub_gazebo_to_odom = rospy.Publisher('/odometry/filtered', Odometry, queue_size = 1)

		self.br = tf.TransformBroadcaster()

		self.waypts = np.load("waypts.npy")

		self.markerVisualization_obj = utils_viz.markerVisualization()
		self.pure_pursuit_obj = utils_pp.purePursuit(self.waypts)

		self.clock_now = 0
		self.clock_last_motion_update = 0
		self.clock_last_rviz_update = 0




	def callback_clock(self, data):
		# for i in data.name:
		# 	print(i)
		self.clock_now = data.clock.secs*1000 + int(data.clock.nsecs/1e6)
		# print(self.clock_now)
		# print(data.clock/1000)

	def callback_odom(self, data):

		r_quat_ros = data.pose.pose.orientation
		r_quaternion_list = [r_quat_ros.x, r_quat_ros.y, r_quat_ros.z, r_quat_ros.w]
		# r_quaternion_list = [r_quaternion.w, r_quaternion.x, r_quaternion.y, r_quaternion.z]
		# r_quaternion = np.quaternion(r_quat_ros.w, r_quat_ros.x, r_quat_ros.y, r_quat_ros.z)
		# r_quaternion = np.quaternion(r_quat_ros.x, r_quat_ros.y, r_quat_ros.z, r_quat_ros.w)
		# # rotation = R.from_quat(quaternion_list)
		# r_euler = quaternion.as_euler_angles(r_quaternion)

		# yaw = np.sign(r_euler[0])*np.sign(r_euler[2])*r_euler[1]
		# print(yaw*180/np.pi)

		# rotation_euler =  rotation.as_euler('zyx', degrees=True)
		(roll, ptch, yaw) = euler_from_quaternion(r_quaternion_list)
		# print(yaw*180/np.pi)



		# self.pure_pursuit_obj.x_pos = data.pose.pose.position.x
		# self.pure_pursuit_obj.y_pos = data.pose.pose.position.y
		# self.pure_pursuit_obj.theta = yaw
		if (self.clock_now - self.clock_last_rviz_update>10):
			self.markerVisualization_obj.publish_marker_waypts(self.waypts)
			self.markerVisualization_obj.publish_lines_waypts(self.waypts)
			self.markerVisualization_obj.publish_marker_robot_pose(data.pose.pose)
			self.markerVisualization_obj.publish_marker_lookahead_circle(data.pose.pose, self.pure_pursuit_obj.lookahead)
			self.markerVisualization_obj.publish_marker_goal(self.pure_pursuit_obj.pg)
			self.markerVisualization_obj.publish_marker_pts_curv(self.waypts,self.pure_pursuit_obj.waypts_curvature)
			self.clock_last_rviz_update = self.clock_now

		if (self.clock_now - self.clock_last_motion_update>10):
			robot_pose = data.pose.pose.position
			self.pure_pursuit_obj.update_pos(robot_pose.x, robot_pose.y, yaw)
			self.pure_pursuit_obj.find_lookahead_pt()
			self.pure_pursuit_obj.find_curvature()
			x_vel, ang_vel = self.pure_pursuit_obj.motion_update()
			self.pure_pursuit_obj.check_reset()
			# print(x_vel, ang_vel)



			twist = Twist()
			twist.linear.x = x_vel
			twist.linear.y = 0
			twist.linear.z = 0

			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = ang_vel

			self.pub_cmd_vel.publish(twist)
			self.pub_curvature.publish(self.pure_pursuit_obj.curvature)
			self.clock_last_motion_update = self.clock_now

	# def callback_gazebo_link_states(self, data):
	# 	# print("hello")
	# 	odom = Odometry()
	# 	data_index = [i for (i, x) in enumerate(data.name) if (x == 'jackal::base_link')]
		
	# 	pose = data.pose[data_index[0]]
	# 	twist = data.twist[data_index[0]]

	# 	odom.pose.pose.position.x = pose.position.x
	# 	odom.pose.pose.position.y = pose.position.y
	# 	odom.pose.pose.orientation.x = pose.orientation.x
	# 	odom.pose.pose.orientation.y = pose.orientation.y
	# 	odom.pose.pose.orientation.z = pose.orientation.z
	# 	odom.pose.pose.orientation.w = pose.orientation.w

	# 	odom.twist.twist.linear = twist.linear
	# 	odom.twist.twist.angular = twist.angular

	# 	odom.header.frame_id = 'odom'
	# 	odom.header.stamp = rospy.Time.now()
	# 	self.pub_gazebo_to_odom.publish(odom)
	# 	self.br.sendTransform(	(pose.position.x, pose.position.y, pose.position.z),
	# 							(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
	# 							rospy.Time.now(), 'base_link', 'odom')

		# r_quat_ros = odom.pose.pose.orientation
		# r_quaternion_list = [r_quat_ros.x, r_quat_ros.y, r_quat_ros.z, r_quat_ros.w]
		# (roll, pitch, yaw) = euler_from_quaternion(r_quaternion_list)

		# self.markerVisualization_obj.publish_marker_waypts(self.waypts)
		# self.markerVisualization_obj.publish_lines_waypts(self.waypts)
		# self.markerVisualization_obj.publish_marker_robot_pose(odom.pose.pose)
		# self.markerVisualization_obj.publish_marker_lookahead(odom.pose.pose, self.pure_pursuit_obj.lookahead)


# def send_cmd_vel():
# 	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
# 	rospy.init_node('pure_pursuit')
# 	rate = rospy.Rate(10) # 10hz

# 	while not rospy.is_shutdown():

# 		twist = Twist()
# 		twist.linear.x = 10
# 		twist.linear.y = 0
# 		twist.linear.z = 0

# 		twist.angular.x = 0
# 		twist.angular.y = 0
# 		twist.angular.z = 0.5

# 		pub.publish(twist)
# 		rate.sleep()


if __name__=="__main__":

	# ===============================================
	# COLLECT WAYPOINTS
	# ===============================================

	waypts = utils_pp.collectPts(num, x_world, y_world, x_fig, y_fig)


	try:
		# send_cmd_vel()	
		rospy.init_node('pure_pursuit_node')
		pure_pursuit_node_obj = pure_pursuit_node_class()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

	# finally:
	# 	twist = Twist()
	# 	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	# 	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	# 	pub.publish(twist)

