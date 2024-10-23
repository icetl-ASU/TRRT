import numpy as np 
from stateSpace import stateSpaceXY

def noCollisionCircle(s1, s2, o):
	# chekc if collide with circle: true if collide
	# s1: state 1
	# s2: state 2
	# o: cricle defined as [x, y , r] where [x, y] is center of circle and r is radius
	# line_length = np.linalg.norm(s2 - s1)
	# if line_length <= 1e-5:
	# 	if np.linalg.norm(s2 - o[0 : 2]) > o[2] and np.linalg.norm(s1 - o[0 : 2]) > o[2]:
	# 		return False
	# 	else:
	# 		return True
	# dot = (((o[0] - s1[0]) * (s2[0] - s1[0]) + (o[1] - s1[1]) * (s2[1] - s1[1]))) \
	# 	/ (line_length ** 2)
	# closest_point = np.array([s1[0] + dot * (s2[0] - s1[0]), \
	# 	s1[1] + dot * (s2[1] - s1[1])])
	# sum_dist = np.linalg.norm(s2 - closest_point) + np.linalg.norm(s1 - closest_point)
	# if not ((sum_dist >= line_length - 0.01) & (sum_dist <= line_length + 0.01)):
	# 	return False

	# return np.linalg.norm(closest_point - o[0:2]) <= o[2]
	d = s2 - s1
	f = s1 - o[0:2]
	r = o[2]
	a = np.dot(d, d)
	if a <= 1e-5:
		if np.linalg.norm(s2 - o[0 : 2]) > o[2] and np.linalg.norm(s1 - o[0 : 2]) > o[2]:
			return False
		else:
			return True
	b = 2 * np.dot(f, d)
	c = np.dot(f, f) - r * r
	temp = b * b - 4 * a * c
	if temp < 0:
		return False
	else:
		temp = temp ** 0.5
		t1 = (-b - temp) / (2 * a)
		t2 = (-b + temp) / (2 * a)
		if (t1 >= 0) & (t1 <= 1):
			return True
		if (t2 >= 0) & (t2 <= 1):
			return True
		return False

# define polygon obstacle (Assume convex for now)
# maybe lay 


class stateValidator:
	def __init__(self, state_space, obs_circle):
		self.state_space = state_space
		self.obstacles_cricle = obs_circle

	def isStateValid(self, state):
		# check if state is valid or not
		inbounds = [state >= self.state_space.state_bounds[:, 0], \
		state <= self.state_space.state_bounds[:, 1]]
		return np.all(inbounds)

	def isMotionValid(self, s1, s2):
		# check if collide with obstacles
		temp_collision = True
		for obs in self.obstacles_cricle:
			if noCollisionCircle(s1, s2, obs):
				temp_collision = False
				break
		return temp_collision

	def isObstacleInTriangle(self, s1, s2, s3):
		x1 = s1[0] 
		y1 = s1[1]
		x2 = s2[0]
		y2 = s2[1]
		x3 = s3[0]
		y3 = s3[1]
		for obs in self.obstacles_cricle:
			x = obs[0]
			y = obs[1]
			if isInside(x1, y1, x2, y2, x3, y3, x, y):
				return True
		return False




def area(x1, y1, x2, y2, x3, y3):
 
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)


def isInside(x1, y1, x2, y2, x3, y3, x, y):
 
    # Calculate area of triangle ABC
    A = area (x1, y1, x2, y2, x3, y3)
 
    # Calculate area of triangle PBC
    A1 = area (x, y, x2, y2, x3, y3)
     
    # Calculate area of triangle PAC
    A2 = area (x1, y1, x, y, x3, y3)
     
    # Calculate area of triangle PAB
    A3 = area (x1, y1, x2, y2, x, y)
     
    # Check if sum of A1, A2 and A3
    # is same as A
    if(A == A1 + A2 + A3):
        return True
    else:
        return False

	## TODO: Add different types of obstacles

# test
# state_bounds = np.array([[0, 200], [0, 200]])
# ss = stateSpaceXY(state_bounds)
# s1 = ss.sampleUniform()
# s2 = ss.sampleUniform()
# obstacles = np.array([[50, 150, 15], [60, 85, 10], [150, 100, 15]])
# print(s1)
# print(s2)
# sv = stateValidator(ss, obstacles)
# print(sv.isMotionValid(s1, s2))

