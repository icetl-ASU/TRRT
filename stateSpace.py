import numpy as np 
from halton import halton

class stateSpaceXY:
	def __init__(self, state_bounds):
		self.num_state_variables = 2
		self.state_bounds = state_bounds

	def dist(self, state1, state2):
		return np.linalg.norm(state2 - state1)

	def sampleUniform(self):
		return np.array([np.random.uniform(self.state_bounds[0][0], self.state_bounds[0][1]), \
			np.random.uniform(self.state_bounds[1][0], self.state_bounds[1][1])])

	def sampleHalton(self, i):
		sample_ratio = halton(i, self.num_state_variables)
		return np.array([self.state_bounds[0][0] + sample_ratio[0] * (self.state_bounds[0][1] - self.state_bounds[0][0]), \
			self.state_bounds[1][0] + sample_ratio[1] * (self.state_bounds[1][1] - self.state_bounds[1][0])])



	## TODO: Add other sampling methods

	## TODO: Add interpolation function

# class stateSpace


# test
# state_bounds = np.array([[0, 10], [0, 10]])
# ss = stateSpaceXY(state_bounds)

# points = ss.sampleHalton(0).reshape(1, 2)

# for i in range(1, 200):
# 	s0 = ss.sampleHalton(i).reshape(1, 2)
# 	points = np.append(points, s0, axis = 0)

# import matplotlib.pyplot as plt
# plt.scatter(points[:, 0], points[:, 1])
# plt.savefig('test.png')
# s1 = ss.sampleUniform()
# s2 = ss.sampleUniform()
# print(s1)
# print(s2)

# dist12 = ss.dist(s1, s2)
# print(dist12)