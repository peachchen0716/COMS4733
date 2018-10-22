#!/usr/bin/env python

import numpy as np 
# import cv2
import math
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

def load_obstacles(object_path):
	'''
	Function to load a list of obstacles.
	The obstacle txt file show points in clockwise order

	Return:
		3d list [[[1, 2], [3, 4], [5, 6]], 
						[[7, 8], [9, 10], [10, 11]]]
	'''
	obstacles = []
	obstacle = []
	with open(object_path) as f:
		numObstacles = int(f.readline())
		coordinates = int(f.readline())
		for i in range(coordinates):
			line = f.readline()
			obstacle.append(list(map(int, line.strip().split(' '))))
		for line in f:
			coordinates = list(map(int, line.strip().split(' ')))
			if len(coordinates) == 1:
				obstacles.append(obstacle)
				obstacle = []
			else:
				obstacle.append(coordinates)
	obstacles.append(obstacle)
	assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
	return obstacles

def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal

def grow_obstacle(obstacles, bot):

	grown_obstacles = []
	
	for i in xrange(len(obstacles)):
		tmp = []
		for v in obstacles[i]:
			for p in bot:
				tmp.append([v[0] + p[0], v[1] + p[1]]) 

		obstacles[i] += tmp
		points = np.array(obstacles[i])
		hull = ConvexHull(points)

		# plt.plot(points[:,0], points[:,1], 'o')
		# for simplex in hull.simplices:
		# 	plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
		# plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
		# print points[hull.vertices, :]
		grown_obstacles.append(points[hull.vertices, :])
		# plt.show()

	return grown_obstacles

class Point:
	def __init__(self, point):
		self.x = point[0]
		self.y = point[1]

	def __str__(self):
		return "(%d, %d)" % (self.x, self.y)

	def __eq__(self, other):
		return self.x == other.x and self.y == other.y

	def __ne__(self, other):
		return not self.__eq__(other)

# Given three colinear points p, q, r, the function checks if 
# point q lies on line segment 'pr' 
def on_segment(p, q, r):
    if q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and \
	   q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y):
       return True
  
    return False
  
# To find orientation of ordered triplet (p, q, r). 
# The function returns following values 
# 0 --> p, q and r are colinear 
# 1 --> Clockwise 
# 2 --> Counterclockwise 
def orientation(p, q, r):
    val = (q.y - p.y) * (r.x - q.x) - \
              (q.x - p.x) * (r.y - q.y)
  
    if val == 0: 
    	return 0  # colinear 
  
    return 1 if val > 0 else 2 # clock or counterclock wise 
  
# The main function that returns True if line segment 'p1q1' 
# and 'p2q2' intersect. 
def do_intersect(p1, q1, p2, q2):

	if p1 == p2 or p1 == q2 or q1 == p2 or q1 == q2:
		return False
    # Find the four orientations needed for general and 
    # special cases 
	o1 = orientation(p1, q1, p2) 
	o2 = orientation(p1, q1, q2) 
	o3 = orientation(p2, q2, p1) 
	o4 = orientation(p2, q2, q1) 
  
    # General case 
	if o1 != o2 and o3 != o4:
		return True 
  
    # Special Cases 
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1 
	if o1 == 0 and on_segment(p1, p2, q1):
		return True 
  
	# p1, q1 and q2 are colinear and q2 lies on segment p1q1 
	if o2 == 0 and on_segment(p1, q2, q1):
		return True 
  
	# p2, q2 and p1 are colinear and p1 lies on segment p2q2 
	if o3 == 0 and on_segment(p2, p1, q2):
		return True 
  
	# p2, q2 and q1 are colinear and q1 lies on segment p2q2 
	if o4 == 0 and on_segment(p2, q1, q2):
		return True 
  
	return False # Doesn't fall in any of the above cases 

def create_vgraph(start, goal, grown_obstacles):

	# get dimension and build a map of index -> vertex
	vertices = {}
	vertices[0] = start
	num_vertices = 1
	for ob in grown_obstacles:
		for v in ob:
			vertices[num_vertices] = v
			num_vertices += 1
	vertices[num_vertices] = goal
	num_vertices += 1
	print num_vertices

	# matrix = np.zeros(shape=(num_vertices, num_vertices))
	matrix = np.full((num_vertices, num_vertices), -1)

	for i in xrange(num_vertices):
		for j in xrange(num_vertices):
			if i == j:
				continue
			if matrix[j][i] != -1:
				matrix[i][j] = matrix[j][i]
				continue
			p1 = Point(vertices[i])
			p2 = Point(vertices[j])

			reachable = True
			for ob in grown_obstacles:
				if reachable == False:
					break
				# loop through obstacle edges
				for k in xrange(ob.shape[0]):
					l = -1 if (k + 1) == ob.shape[0] else k + 1
					p3 = Point(ob[k])
					p4 = Point(ob[l])
					if do_intersect(p1, p2, p3, p4):
						# print i," ", j, "segment collide"

						reachable = False
						break
			if reachable:
				matrix[i][j] = math.hypot(p1.x - p2.x, p1.y - p2.y)
				# print "from ", p1, " to ", p2, " dst = ", matrix[i][j]

	return matrix


if __name__ == "__main__":

	bot = [[18, 18], [18, -18], [-18, 18], [-18, -18]]

	obstacles = load_obstacles("../data/world_obstacles.txt")
	
	grown_obstacles = grow_obstacle(obstacles, bot)
	
	# print grown_obstacles

	goal = load_goal("../data/goal.txt")
	start = [0, 0]

	# print goal

	matrix = create_vgraph(start, goal, grown_obstacles)
	print matrix





