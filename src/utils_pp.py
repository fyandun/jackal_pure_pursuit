import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation


def collectPts(num, x_world, y_world, x_fig, y_fig):
	fig = plt.figure(figsize=(x_fig,y_fig))
	ax1 = fig.add_subplot(1,1,1)
	ax1.set_ylim(0, y_world)
	ax1.set_xlim(0, x_world)
	ax1.clear()
	ax1.set_ylim(0, 10)
	ax1.set_xlim(0, 10)	
	waypts = np.asarray(plt.ginput(n=num, timeout=30, ))
	waypts = waypts.round(4)
	np.save("waypts", waypts)
	plt.close()
	


class purePersuit():


	def __init__(self,waypts):

		self.waypts = waypts
		self.num_waypts = waypts.shape[0]
		self.lookahead = 1.5

		self.index = 0
		self.running_index = 0

		self.pt_pos = waypts[0]

		self.x_pos = self.pt_pos[0]
		self.y_pos = self.pt_pos[1]
		self.theta = self.angle_bw_2lines(np.array([self.waypts[0][0]+1,self.waypts[0][1]]),self.waypts[0], self.waypts[1])
		self.x_head_pos = 0
		self.y_head_pos = 0

		self.pg = self.waypts[0]
		self.arc_c = self.waypts[0]
		self.curvature = 0

		self.dt = 0.050

		self.vel=1.5
		self.vel_left = 0
		self.vel_right = 0
		self.target_vel = self.vel
		self.ang_vel = 0
		self.ang_vel_thresh = 2

		self.bot_width =0.4

		self.x_navPath = []
		self.y_navPath = []
		# print(self.x_pos, self.y_pos)

		self.reset_flag = True

		self.waypts_curvature = [0]
		self.find_pt_to_pt_curvature()

		print("initialize", self.x_pos)


	def find_pt_to_pt_curvature(self):
		print(self.num_waypts)

		for i in range(1,self.num_waypts-1):
			x1 = self.waypts[i-1][0]
			y1 = self.waypts[i-1][1]
			x2 = self.waypts[i][0]
			y2 = self.waypts[i][1]
			x3 = self.waypts[i+1][0]
			y3 = self.waypts[i+1][1]

			m1 = x2-x1
			m2 = y2-y1
			n1 = x3-x2
			n2 = y3-y2
			m3 = ((x2*x2+y2*y2)-(x1*x1+y1*y1))/2
			n3 = ((x3*x3+y3*y3)-(x2*x2+y2*y2))/2

			a = (m3*n2-m2*n3)/(m1*n2-m2*n1)
			b = (m3*n1-m1*n3)/(m2*n1-m1*n2)

			r = np.sqrt((x2-a)*(x2-a)+(y2-b)*(y2-b))
			curvature = 1/r
			self.waypts_curvature.append(curvature)
			print("point", i+1, curvature)

		self.waypts_curvature.append(0)
		print(self.waypts_curvature)


	def reset_pos(self):
		self.x_pos = self.pt_pos[0]
		self.y_pos = self.pt_pos[1]
		self.theta = self.angle_bw_2lines(np.array([self.waypts[0][0]+1,self.waypts[0][1]]),self.waypts[0], self.waypts[1])
		self.pg = self.waypts[0]
		self.index = 0
		self.running_index = 0



	def get_pos(self):
		return (self.x_pos, self.y_pos, self.theta)



	def update_pos(self, x, y, theta):
		self.x_pos = x
		self.y_pos = y
		self.theta = theta


	def get_head_pos(self):
		self.x_head_pos = self.x_pos+self.lookahead*np.cos(self.theta)
		self.y_head_pos = self.y_pos+self.lookahead*np.sin(self.theta)
		return(self.x_head_pos, self.y_head_pos)



	def angle_bw_2lines(self, a, b, c):
		ba = a - b
		bc = c - b
		cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
		angle = np.arccos(cosine_angle)
		# print(np.degrees(angle))
		return angle



	def calculate_dist(self, pt):
		return np.linalg.norm(self.xy_pos()-pt)



	def xy_pos(self):
		return np.array([self.x_pos, self.y_pos])



	def find_closest_point(self):
		min_dist = 9999;
		for i in range(self.num_waypts):
			# print(self.calculate_dist(self.waypts[i]))
			if (min_dist>self.calculate_dist(self.waypts[i])):
				min_dist = self.calculate_dist(self.waypts[i])
				self.index = i
		# print(self.index)
		return self.waypts[self.index]




	def find_lookahead_pt(self):
		if not self.reset_flag:
			tmp_index = int(np.floor(self.running_index))
			if (tmp_index+1<self.num_waypts):

				for i in range(self.num_waypts-1):
					# print("index_check", tmp_index-2)

					if (tmp_index+1>=self.num_waypts):
						break

					# print("loop", i, "tmp_index", tmp_index, tmp_index+1)
					path = self.waypts[tmp_index+1] - self.waypts[tmp_index]
					pc = self.waypts[tmp_index] - self.xy_pos()
					# print("waypts", self.waypts[tmp_index])
					# print("xy_pos", self.xy_pos())
					# print("x_pos", self.x_pos)
					# print("y_pos", self.y_pos)
					a = np.dot(path,path)
					b = 2*(np.dot(path,pc))
					c = np.dot(pc,pc) - np.square(self.lookahead)
					# print("a b c" ,a,b,c)
					# print("path" ,path)
					# print("pc" ,pc)

					t = np.roots([a,b,c])
					t = np.max(t)
					if ((t<=1) and (t>=0) and np.isreal(t) and (tmp_index+t>self.running_index)):
						self.pg=self.waypts[tmp_index] + t*path
						self.running_index = tmp_index+t
						break
					tmp_index = tmp_index+1
		else:
			self.pg = self.waypts[0]
			if self.calculate_dist(self.waypts[0])<0.5:
				self.reset_flag = False


		return self.pg



	def find_curvature(self):

		a = -np.tan(self.theta)
		b = 1
		c = self.x_pos*np.tan(self.theta) - self.y_pos

		N = abs(a*self.pg[0] + b*self.pg[1] + c)/np.sqrt(np.square(a)+np.square(b))
		# print(N)
		sign_pos = self.pg-self.xy_pos()
		# print(sign_pos)
		side = sign_pos[0]*np.sin(self.theta) - sign_pos[1]*np.cos(self.theta)
		side = np.sign(side)
		# print("side", side)
		# # print("side" , side)

		self.curvature = 2*N/np.square(self.lookahead)*side*-1



	def motion_update(self):

		
		# upcoming_pt = int(np.ceil(self.running_index))
		# self.target_vel = self.vel*1.00*(1-self.waypts_curvature[upcoming_pt])
		# print("running_index",np.ceil(self.running_index), "vel", self.target_vel)

		tmp_curv = np.clip(self.curvature, -1, +1)
		vel_decay = np.absolute(tmp_curv)
		# print("curvature", tmp_curv, self.curvature)
		vel_decay = 0.5 + (1-vel_decay)*0.5
		print("vel_decay", vel_decay)



		self.ang_vel = self.target_vel*self.curvature
		self.ang_vel = self.target_vel*self.curvature*vel_decay



		# self.theta = self.theta + self.ang_vel*self.dt
		# self.x_pos = self.x_pos+self.target_vel*np.cos(self.theta)*self.dt
		# self.y_pos = self.y_pos+self.target_vel*np.sin(self.theta)*self.dt

		# self.x_navPath.append(self.x_pos)
		# self.y_navPath.append(self.y_pos)
		self.ang_vel = np.clip(self.ang_vel, -self.ang_vel_thresh, self.ang_vel_thresh)


		return self.target_vel*vel_decay, self.ang_vel



	def check_reset(self):
		if (np.linalg.norm(self.xy_pos()-self.waypts[-1])<0.2):
			self.reset_flag = True
			# self.x_pos = self.pt_pos[0]
			# self.y_pos = self.pt_pos[1]
			# self.theta = self.angle_bw_2lines(np.array([self.waypts[0][0]+1,self.waypts[0][1]]),self.waypts[0], self.waypts[1])
			self.pg = self.waypts[0]
			self.index = 0
			self.running_index = 0
			self.x_navPath = []
			self.y_navPath = []


