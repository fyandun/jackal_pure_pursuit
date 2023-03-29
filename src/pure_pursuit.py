#!/usr/bin/env python3

from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time

import sys
import os

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Clock
from  tf.transformations import euler_from_quaternion

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation

from utils_pp import PurePursuitController
import utils_viz
import tf

class pure_pursuit_node_class:
	def __init__(self):

		# Subscribers
		self.sub_clock = rospy.Subscriber('/clock', Clock, self.callback_clock)
		self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)

		# Publishers
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.pub_curvature = rospy.Publisher('curvature', Float32, queue_size = 1)

		self.br = tf.TransformBroadcaster()

		self.waypoint_path = rospy.get_param('/waypoints_path','waypts.npy')
		self.waypts = np.load(self.waypoint_path)

		rospy.loginfo_once(f'Initialized Pure Pursuit Controller with {self.waypts.shape[0]} waypoints')

		self.markerVisualization_obj = utils_viz.markerVisualization()

		lookahead_dist = rospy.get_param('/lookahead',1.5)
		self.ppc = PurePursuitController(self.waypts, lookahead_dist)

		self.clock_now = 0
		self.clock_last_motion_update = 0
		self.clock_last_rviz_update = 0


	def callback_clock(self, data):
		self.clock_now = data.clock.secs*1000 + int(data.clock.nsecs/1e6)

	def callback_odom(self, data):
		robot_pose = data.pose.pose.position
		robot_orientation = data.pose.pose.orientation
		r_quaternion_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
		_, _, yaw = euler_from_quaternion(r_quaternion_list)
		# _, _, yaw = euler_from_quaternion(r_quaternion_list,'xyzs')
		# r_quaternion_list = [r_quaternion.w, r_quaternion.x, r_quaternion.y, r_quaternion.z]

		if (self.clock_now - self.clock_last_rviz_update > 10):
			self.markerVisualization_obj.publish_marker_waypts(self.waypts)
			self.markerVisualization_obj.publish_lines_waypts(self.waypts)
			self.markerVisualization_obj.publish_marker_robot_pose(data.pose.pose)
			self.markerVisualization_obj.publish_marker_lookahead_circle(data.pose.pose, self.ppc.lookahead)
			self.markerVisualization_obj.publish_marker_goal(self.ppc.pg)
			self.markerVisualization_obj.publish_marker_pts_curv(self.waypts,self.ppc.waypts_curvature)
			self.clock_last_rviz_update = self.clock_now

		if (self.clock_now - self.clock_last_motion_update > 10):
			self.ppc.update_pos(robot_pose.x, robot_pose.y, yaw)
			self.ppc.find_lookahead_pt()
			self.ppc.find_curvature()
			x_vel, ang_vel = self.ppc.motion_update()
			self.ppc.check_reset()

			# twist = Twist(
			# 	linear=Vector3(x_vel, 0, 0),
			# 	angular=Vector3(0, 0, ang_vel)
			# )
			twist = Twist(
				linear=Vector3(x_vel, 0, 0),
				angular=Vector3(0, 0, self.ppc.curvature)
			)

			self.pub_cmd_vel.publish(twist)
			# self.pub_curvature.publish(self.ppc.curvature)
			self.clock_last_motion_update = self.clock_now


if __name__ == '__main__':

	try:
		rospy.init_node('pure_pursuit_node')
		pure_pursuit_node_obj = pure_pursuit_node_class()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

