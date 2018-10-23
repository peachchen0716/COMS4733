#!/usr/bin/env python

import rospy
import numpy as np 
import math
import heapq
from scipy.spatial import ConvexHull
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class VGraph():
	def __init__(self):
		rospy.init_node('vgraph_test', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		self.init_markers()

	def draw_marker(self):
		while not rospy.is_shutdown():
			# Update the marker display
			self.marker_pub.publish(self.markers)


	def grow_obstacle(self, obstacles, bot):

		grown_obstacles = []
	
		for i in xrange(len(obstacles)):
			tmp = []
			for v in obstacles[i]:
				for p in bot:
					tmp.append([v[0] + p[0], v[1] + p[1]]) 

			obstacles[i] += tmp
			points = np.array(obstacles[i])
			hull = ConvexHull(points)

			grown_obstacles.append(points[hull.vertices, :])

		print "start to grow obstacle"

		for obstacle in grown_obstacles:
			first = Point()
			for i in xrange(len(obstacle)):
				p = Point()
				p.x = float(obstacle[i][0]) / 100
				p.y = float(obstacle[i][1]) / 100
				self.markers.points.append(p)
				if i != 0:
					self.markers.points.append(p)
				else:
					first = p
			self.markers.points.append(first)

		return grown_obstacles



	def create_graph(self, start, goal, grown_obstacles):
		# get number of vetices
		# and build a map of index -> vertex
		vertices = {}
		colors = {}
		vertices[0] = MyPoint(start)	
		colors[0] = -1
		num_vertices = 1
		for i in xrange(len(grown_obstacles)):
			for v in grown_obstacles[i]:
				vertices[num_vertices] = MyPoint(v)
				colors[num_vertices] = i
				num_vertices += 1
		vertices[num_vertices] = MyPoint(goal)
		colors[num_vertices] = -2

		num_vertices += 1

		matrix = np.full((num_vertices, num_vertices), -1)

		# for each pair of vertices, find the direct distance
		# dst is -1 if they are not connected
		for i in xrange(num_vertices):
			for j in xrange(num_vertices):
				if i == j or colors[i] == colors[j]:
					continue
				if matrix[j][i] != -1:
					matrix[i][j] = matrix[j][i]
					continue
				p1 = vertices[i]
				p2 = vertices[j]

				reachable = True
				for ob in grown_obstacles:
					if reachable == False:
						break
					# loop through obstacle edges
					for k in xrange(len(ob)):
						l = 0 if (k + 1) == len(ob) else k + 1

						p3 = MyPoint(ob[k])
						p4 = MyPoint(ob[l])
						if p3.x == 168 and p3.y == -18 \
							and p4.x == 168 and p4.y == 18:
							print "test"
						if do_intersect(p1, p2, p3, p4):
							reachable = False
							break
				if reachable:
					matrix[i][j] = math.hypot(p1.x - p2.x, p1.y - p2.y)
					p = Point(float(p1.x) / 100, float(p1.y) / 100, 0)
					q = Point(float(p2.x) / 100, float(p2.y) / 100, 0)
					self.markers.points.append(p)
					self.markers.points.append(q)
					
		return matrix, vertices

	def draw_edges(self, matrix):
		return

	def init_markers(self):
		marker_scale = 0.01
		marker_lifetime = 0
		marker_ns = 'vgraph_ns'
		marker_id = 0
		marker_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
		self.marker_pub = rospy.Publisher('vgraph_markers', Marker, \
			queue_size=5)

		# Initialize the marker points list.
		self.markers = Marker()
		self.markers.ns = marker_ns
		self.markers.id = marker_id
		self.markers.type = 5
		self.markers.action = Marker.ADD
		self.markers.lifetime = rospy.Duration(marker_lifetime)
		self.markers.scale.x = marker_scale
		self.markers.scale.y = marker_scale
		self.markers.color.r = marker_color['r']
		self.markers.color.g = marker_color['g']
		self.markers.color.b = marker_color['b']
		self.markers.color.a = marker_color['a']
		self.markers.header.frame_id = 'map'
		self.markers.header.stamp = rospy.Time.now()
		self.markers.points = list()

	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		# Cancel any active goals
		# self.move_base.cancel_goal()
		# rospy.sleep(2)
		# Stop the robot
		# self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

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

class MyPoint:
	def __init__(self, point):
		self.x = point[0]
		self.y = point[1]

	def __str__(self):
		return "(%d, %d)" % (self.x, self.y)

	def __eq__(self, other):
		return self.x == other.x and self.y == other.y

	def __ne__(self, other):
		return not self.__eq__(other)
	
	def dst_from(self, other):
		return math.hypot(self.x - other.x, self.y - other.y)

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
	count = 0
	if p1 == p2 or p1 == q2:
		count += 1
	if q1 == p2 or q1 == q2:
		count += 1
	if count == 1:
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



def dijkstra(matrix, start, goal):
	return

def aster(matrix, start, goal, vertices):

	# initial set up
	visited = [False] * len(matrix)
	g = {}
	h = {}
	parents = {}
	pq = []	

	# push start into priority queue
	g[start] = 0
	h[start] = vertices[start].dst_from(vertices[goal])
	heapq.heappush(pq, (g[start] + h[start], start))
	visited[start] = True

	while len(pq) != 0:
		dst, cur = heapq.heappop(pq)
		for i in xrange(len(matrix[cur])):
			if matrix[cur][i] == -1:
				continue
			# find goal
			if i == goal:
				parents[i] = cur
				return parents
			# if not visited
			if visited[i] == False:
				pos = -1
				g[i] = min(g[i], g[cur] + matrix[cur][i])
				h[i] = min(h[i], vertices[i].dst_from(vertices[goal]))
				for j in xrange(len(pq)):
					if pq[j][1] == i:
						pos = j
				if pos != -1:
					heapq.heappush(pq, (g[i] + h[i], i))
					parents[i] = cur
				else:
					tmp_tuple = pq[pos]
					if tmp_tuple > g[i] + h[i]:
						pq[pos] = (g[i] + h[i], i)
						heapq.heapify(pq)
		visited[cur] = True
	return parents


if __name__ == "__main__":

	bot = [[18, 18], [18, -18], [-18, 18], [-18, -18]]

	obstacles = load_obstacles("../data/world_obstacles.txt")
	
	vgraph = VGraph()
	grown_obstacles = vgraph.grow_obstacle(obstacles, bot)

	for ob in grown_obstacles:
		print ob
	goal = load_goal("../data/goal.txt")
	start = [0, 0]

	matrix, vertices = vgraph.create_graph(start, goal, grown_obstacles)

	vgraph.draw_marker()
	dijkstra(matrix, 0, len(matrix) - 1, vertices)





