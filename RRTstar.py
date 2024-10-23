import numpy as np 
from stateSpace import stateSpaceXY
from stateValidator import stateValidator
import matplotlib.pyplot as plt

class RRTstar:
	def __init__(self, start, goal, ss, sv, \
		max_iter = 500, expand_range = 20, \
		ball_radius = 200):
		"""
		start: start position (num_agents * 2)
		goal: target position [x, y]
		ss: state space 
		sv: state validator 
		"""
		self.points = start 
		self.goal = goal
		self.ss = ss 
		self.sv = sv 
		self.max_iter = max_iter
		self.expand_range = expand_range
		self.ball_radius = ball_radius
		node_start = Node()
		node_start.parent = -1
		node_start.cost = 0
		self.node_list = [node_start]
		
	def planning(self):
		for i in range(self.max_iter):
			# new_pos = self.ss.sampleHalton(i)	
			new_pos = self.ss.sampleUniform()	
			while not self.sv.isMotionValid(new_pos, new_pos):
				new_pos = self.ss.sampleUniform()
			self.planning_point(new_pos)
		if_reached = self.planning_point(self.goal)
		return if_reached


	def planning_point(self, new_pos):
		"""
		main planning algorithm
		"""
		# for i in range(self.max_iter):
		# 	if np.random.rand() > 0.99:
		# 		new_pos = self.goal
		# 	else:
		# 		new_pos = self.ss.sampleUniform()
			# while not self.sv.isStateValid(new_pos, new_pos):
			# 	new_pos = self.ss.sampleUniform()
	
		temp_dist = np.Inf
		temp_near_list = []

		num_node = len(self.node_list)
		# curr_ball_radius = (np.log(num_node + 1) / num_node) ** 0.5 \
		# 	* self.ball_radius
		curr_ball_radius = self.ball_radius

		diff = np.linalg.norm(self.points - new_pos, axis = 1)
		temp_nearest = np.argmin(diff)

		if diff[temp_nearest] < temp_dist:
			temp_id = temp_nearest
			temp_dist = diff[temp_nearest] 
		temp_near_list.append(np.where(diff <= curr_ball_radius)[0])
		nearest_pos = self.points[temp_id, :]
		new_pos = self.steer(nearest_pos, new_pos)

		if self.sv.isMotionValid(new_pos, nearest_pos):
			new_node, new_point = self.calculate_cost(temp_id, new_pos)
			num_list = temp_near_list[0].shape[0]
			for k in range(num_list):
				ind = temp_near_list[0][k]
				near_pos = self.points[ind, :]
				if self.sv.isMotionValid(new_pos, near_pos):
					temp_node, temp_point = self.calculate_cost(ind, new_pos)
					if temp_node.cost < new_node.cost:
						new_node = temp_node
						# new_node.parent = ind
						new_point = temp_point
			self.node_list.append(new_node)
			self.points = np.append(self.points, new_point.reshape(1, 2), axis = 0)

			# Rewiring
			new_node_id = len(self.node_list) - 1
			for k in range(num_list):
				ind = temp_near_list[0][k]
				near_pos = self.points[ind, :]
				if self.sv.isMotionValid(new_pos, near_pos):
					temp_cost = new_node.cost + calculate_distance(new_pos, near_pos)
					if temp_cost < self.node_list[ind].cost + 0.1:
						self.node_list[ind].parent = new_node_id
						self.node_list[ind].cost = temp_cost
			return True
		else:
			return False



	def calculate_cost(self, node_id, new_pos):
		node = self.node_list[node_id]
		new_node = Node()
		new_node.parent = node_id
		new_node.cost = node.cost + calculate_distance(self.points[node_id, :], new_pos)

		# new_point = []
		return new_node, np.array(new_pos)

	def steer(self, start_point, end_point):
		d = np.linalg.norm(end_point - start_point)
		if d > self.expand_range:
			end_point = start_point + self.expand_range / d * (end_point \
				-start_point)
		return end_point	

class Node:
	def __init__(self):
		self.cost = 0		

def calculate_distance(x1, x2):
	return np.linalg.norm(x1 - x2)


def main():
	# Initialize the map
	yMax = 200
	xMax = 200

	state_bounds = np.array([[0, xMax], [0, yMax]])
	ss = stateSpaceXY(state_bounds)
	# Currently circle
	obstacles = np.array([[50, 150, 15], [60, 85, 10], [150, 100, 15]])
	sv = stateValidator(ss, obstacles)
	start = np.array([[20, 150]])
	goal = np.array([175, 175])

	# Initialize setting
	num_agents = 3
	max_L = 280
	move_weight = [1, 2, 1]
	max_l = max_L / (num_agents - 1)
	planner = RRTstar(start, goal, ss, sv)
	# print(start[:, 1, :])
	if_reached = planner.planning()

	# plot the traj
	# plot obstacles
	num_obs = obstacles.shape[0]
	obs_plot = []
	for i in range(num_obs):
		obs_plot.append(plt.Circle((obstacles[i, 0], obstacles[i, 1]), obstacles[i, 2]))

	fig, ax = plt.subplots()
	for i in range(num_obs):
		ax.add_patch(obs_plot[i])

	color = ['r', 'k', 'b']
	# print(if_reached)
	if if_reached:
		curr_node = planner.node_list[-1]
		curr_id = -1
		while curr_node.parent != -1:
			parent_id = curr_node.parent
			# for j in range(num_agents - 1):
			# 	ax.plot()
			# for j in range(num_agents):
			x_pos = [planner.points[curr_id, 0], \
				planner.points[parent_id, 0]]
			y_pos = [planner.points[curr_id, 1], \
				planner.points[parent_id, 1]]
			ax.plot(x_pos, y_pos, color[0], linewidth = 2)
			curr_id = parent_id
			curr_node = planner.node_list[curr_id]
	else:
		print('Try increase max iterations')

	plt.ylim(0, yMax)
	plt.xlim(0, xMax)
	fig.savefig('plotresult.png')
	# # planner.node_list[-1].mid_points[0].shape[0]
	# temp_node= planner.node_list[-1]
	# print(temp_node.mid_points)
	# short_node = planner.shortening(temp_node)
	# print(short_node.mid_points)

if __name__ == "__main__":
    main()