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


def dist(u,v):
	return np.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)

class PurePursuitController():

	def __init__(self,waypts,lookahead):
		self.waypts = waypts
		self.num_waypts = waypts.shape[0]
		self.lookahead = lookahead

		self.index = 0
		self.running_index = 0

		self.pt_pos = waypts[0]

		self.x_pos = self.pt_pos[0]
		self.y_pos = self.pt_pos[1]
		self.theta = self.angle_bw_2lines(np.array([self.waypts[0][0]+1,self.waypts[0][1]]),self.waypts[0], self.waypts[1])
		self.x_head_pos = 0
		self.y_head_pos = 0

		self.cur_goal_point = self.waypts[0]
		self.arc_c = self.waypts[0]
		self.curvature = 0

		self.dt = 0.050

		self.vel = 1.5
		self.target_vel = self.vel
		self.ang_vel = 0
		self.ang_vel_thresh = 2

		self.reset_flag = True

		self.waypts_curvature = [0]
		self.find_pt_to_pt_curvature()

	def find_pt_to_pt_curvature(self):
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

		self.waypts_curvature.append(0)
		print(self.waypts_curvature)

	def calc_curv(self,tx,ty):
		k = []
		for i in range(2,len(tx)):
			u = tx[i-2], ty[i-2]
			v = tx[i-1], ty[i-1]
			w = tx[i], ty[i]

			num = dist(u,v)*dist(v,w)*dist(w,u)
			den = u[0]*(v[1]-w[1]) + v[0]*(w[1]-u[1]) + w[0]*(u[1]-v[1])

			k.append(2*den/num)
		return np.array(k)

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
		return angle

	def calculate_dist(self, pt):
		return np.linalg.norm(self.xy_pos()-pt)

	def xy_pos(self):
		return np.array([self.x_pos, self.y_pos])

	def find_closest_point(self):
		min_dist = 9999
		for i, pt in enumerate(self.num_waypts):
			cur_dist = self.calculate_dist(pt)
			if (min_dist > cur_dist):
				min_dist = cur_dist
				self.index = i
		return self.waypts[self.index]

	def find_lookahead_pt(self):
		if self.reset_flag:
			self.cur_goal_point = self.waypts[0]
			if self.calculate_dist(self.waypts[0]) < 0.5:
				self.reset_flag = False
			return self.cur_goal_point

		tmp_index = int(np.floor(self.running_index))
		while tmp_index + 1 < self.num_waypts:
			path = self.waypts[tmp_index+1] - self.waypts[tmp_index]
			pc = self.waypts[tmp_index] - self.xy_pos()

			a = np.dot(path,path)
			b = 2*(np.dot(path,pc))
			c = np.dot(pc,pc) - np.square(self.lookahead)

			t = np.max(np.roots([a,b,c]))
			if ((t <= 1) and (t >= 0) and np.isreal(t) and (tmp_index + t > self.running_index)):
				self.cur_goal_point = self.waypts[tmp_index] + t*path
				self.running_index = tmp_index+t
				break
			tmp_index += 1

		return self.cur_goal_point

	def find_curvature(self):
		a = -np.tan(self.theta)
		b = 1
		c = self.x_pos*np.tan(self.theta) - self.y_pos

		N = abs(a*self.cur_goal_point[0] + b*self.cur_goal_point[1] + c)/np.sqrt(np.square(a)+np.square(b))

		sign_pos = self.cur_goal_point-self.xy_pos()

		side = sign_pos[0]*np.sin(self.theta) - sign_pos[1]*np.cos(self.theta)
		side = np.sign(side)

		self.curvature = 2*N/np.square(self.lookahead)*side

	def motion_update(self):		
		# upcoming_pt = int(np.ceil(self.running_index))
		# self.target_vel = self.vel*1.00*(1-self.waypts_curvature[upcoming_pt])

		tmp_curv = np.clip(self.curvature, -1, +1)
		vel_decay = np.absolute(tmp_curv)
		vel_decay = 0.5 + (1-vel_decay)*0.5

		self.ang_vel = self.target_vel*self.curvature*vel_decay

		# self.theta = self.theta + self.ang_vel*self.dt
		# self.x_pos = self.x_pos+self.target_vel*np.cos(self.theta)*self.dt
		# self.y_pos = self.y_pos+self.target_vel*np.sin(self.theta)*self.dt

		self.ang_vel = np.clip(self.ang_vel, -self.ang_vel_thresh, self.ang_vel_thresh)

		return self.target_vel*vel_decay, self.ang_vel

	def check_reset(self):
		if (np.linalg.norm(self.xy_pos()-self.waypts[-1])<0.2):
			self.reset_flag = True
			# self.x_pos = self.pt_pos[0]
			# self.y_pos = self.pt_pos[1]
			# self.theta = self.angle_bw_2lines(np.array([self.waypts[0][0]+1,self.waypts[0][1]]),self.waypts[0], self.waypts[1])
			self.cur_goal_point = self.waypts[0]
			self.index = 0
			self.running_index = 0
			print('>>>>>>>>>>>>>>>>>>TRIGGERED RESET')


