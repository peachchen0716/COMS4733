#!/usr/bin/env python

import rospy
import numpy as np 
import heapq
import math
import tf
from math import radians, copysign, sqrt, pow, pi
from geometry_msgs.msg import Twist, Point, Quaternion
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from scipy.spatial import ConvexHull
from visualization_msgs.msg import Marker, MarkerArray

class VGraph():
	def __init__(self):
		rospy.init_node('vgraph_test', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		self.init_markers()
		self.init_markers2()
		self.init_marker_array()

		self.init_bot()

		self.tf_listener = tf.TransformListener()
		rospy.sleep(2)

		self.odom_frame = '/odom'
		try:
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame,'/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")


	def init_marker_array(self):
		self.marker_array_pub = rospy.Publisher('visualization_marker_array', \
			MarkerArray, queue_size=5)
		self.marker_array = MarkerArray()

		self.marker_array.markers.append(self.markers)
		self.marker_array.markers.append(self.markers2)

	def get_odom(self):
		try:
			(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame,self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return
		return (Point(*trans), quat_to_angle(Quaternion(*rot)))

	def init_markers(self):
		marker_scale = 0.02
		marker_lifetime = 0
		marker_ns = 'vgraph_ns'
		marker_id = 2
		marker_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
		# self.marker_pub = rospy.Publisher('vgraph_markers', Marker, \
		# 	queue_size=5)

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

	def init_markers2(self):
		marker_scale = 0.02
		marker_lifetime = 0
		marker_ns = 'vgraph_ns'
		marker_id = 1
		marker_color = {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}
		# self.marker_pub = rospy.Publisher('vgraph_markers', Marker, \
		# 	queue_size=5)

		self.markers2 = Marker()
		self.markers2.ns = marker_ns
		self.markers2.id = marker_id
		self.markers2.type = 5
		self.markers2.action = Marker.ADD
		self.markers2.lifetime = rospy.Duration(marker_lifetime)
		self.markers2.scale.x = marker_scale
		self.markers2.scale.y = marker_scale
		self.markers2.color.r = marker_color['r']
		self.markers2.color.g = marker_color['g']
		self.markers2.color.b = marker_color['b']
		self.markers2.color.a = marker_color['a']
		self.markers2.header.frame_id = 'map'
		self.markers2.header.stamp = rospy.Time.now()
		self.markers2.points = list()

	def init_bot(self):
		# rospy.init_node("my_bot", anonymous=False)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

		self.rate = 20
		self.r = rospy.Rate(self.rate)

		#Speeds
		linear_speed = 0.2
		angular_speed = 1.0

	def translate(self, dist):
		goal_distance = dist
		linear_speed = 0.1
		if dist < 0:
			linear_speed *= -1
		linear_duration = goal_distance/linear_speed


		move_cmd = Twist()
		move_cmd.linear.x = linear_speed
		ticks = int(linear_duration * self.rate)
		for t in range(ticks):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()

		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(0.4)

	def rotate(self, deg):
		#goal_angle = deg * pi / 180.0
		goal_angle = deg
		angular_speed = 0.1
		if deg < 0:
			angular_speed *= -1
		angular_duration = goal_angle / angular_speed

		move_cmd = Twist()
		move_cmd.angular.z = angular_speed

		ticks = int(angular_duration * self.rate)
		for t in range(ticks):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()

		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)

		rospy.sleep(1)

		self.cmd_vel.publish(Twist())

	def backtrack(self, path, vertices):

		pos = len(path) - 1
		shortest_path = []
		p1 = vertices[pos]
		p = Point(float(p1.x) / 100, float(p1.y) / 100, 0)
		self.markers2.points.append(p)
		shortest_path.append(p)

		while pos != 0:
			pos = path[pos]
			p1 = vertices[pos]
			p = Point(float(p1.x) / 100, float(p1.y) / 100, 0)
			self.markers2.points.append(p)
			self.markers2.points.append(p)
			shortest_path.append(p)

		p1 = vertices[0]
		p = Point(float(p1.x) / 100, float(p1.y) / 100, 0)
		self.markers2.points.append(p)
		shortest_path.append(p)

		return shortest_path

	def draw_marker(self):
		while not rospy.is_shutdown():
			# Update the marker display
			# self.marker_pub.publish(self.markers)
			# self.marker_pub.publish(self.markers2)
			self.marker_array_pub.publish(self.marker_array)
			rospy.sleep(2)
			break


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

		print "start growing obstacle"

		for obstacle in grown_obstacles:
			first = Point()
			for i in xrange(len(obstacle)):
				p = Point()
				p.x = float(obstacle[i][0]) / 100
				p.y = float(obstacle[i][1]) / 100
				self.markers.points.append(p)
				if i != 0:
					self.markers.points.append(p)
					j = 1
				else:
					first = p
			self.markers.points.append(first)

		return grown_obstacles


	def create_graph(self, start, goal, grown_obstacles):
		print "start creating graph"

		# get number of vetices
		# a map of index -> vertex
		vertices = {}
		# a map of index -> # of obstacle
		colors = {}

		vertices[0] = MyPoint(start)	
		colors[0] = -1
		num_v = 1

		for i in xrange(len(grown_obstacles)):
			for v in grown_obstacles[i]:
				vertices[num_v] = MyPoint(v)
				colors[num_v] = i
				num_v += 1

		vertices[num_v] = MyPoint(goal)
		colors[num_v] = -2
		num_v += 1

		matrix = np.full((num_v, num_v), -1)

		# add all obstacle edges into adjancency matrix
		count = 1
		for ob in grown_obstacles:
			for k in xrange(len(ob)):
				l = 0 if (k + 1) == len(ob) else k + 1 # edge case			
				p1 = vertices[count + k]
				p2 = vertices[count + l]
				dst = math.hypot(p1.x - p2.x, p1.y - p2.y)
				matrix[count + k][count + l] = dst
				matrix[count + l][count + k] = dst
			count += len(ob)

		# for each pair of vertices, find distance
		# dst is -1 if they are not connected
		for i in xrange(num_v):
			for j in xrange(num_v):
				# if i == j or (colors[i] == colors[j] and abs(i - j) > 1):
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
						l = 0 if (k + 1) == len(ob) else k + 1 # edge case
						p3 = MyPoint(ob[k])
						p4 = MyPoint(ob[l])
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
	if count == 2:
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


def dijkstra(matrix, vertices, start, goal):
	print "start dijkstra search"
	vertices_c = vertices.copy()
	visited = [0 for i in range(len(vertices))]
	path = [None for i in range(len(vertices))]
	distance = [np.Infinity for i in range(len(vertices))]

	distance[start] = 0

	while np.sum(visited) != len(vertices):
		min = np.Infinity
		for v in vertices_c:
			# if visited[v] == False and distance[v] < min:
			if distance[v] < min:
				min = distance[v]
				u = v

		visited[u] = 1
		del vertices_c[u]

		if u == goal:
			break

		for v in vertices_c:
			if matrix[u][v] == -1:
				continue
			
			extended_path = distance[u] + matrix[u][v]
			if extended_path < distance[v]:
				distance[v] = extended_path
				path[v] = u

	return path

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

def angle(p1, p2):
	if p2.x - p1.x == 0:
		return 0
	slope = (p2.y - p1.y)/(p2.x - p1.x)
	angle = np.arctan(slope)
	return angle


def follow_path(path):
	vgraph = VGraph()
	currentpos, currentangle = vgraph.get_odom()
	print currentpos, currentangle

	path.reverse()
	old_angle = 0
	i = 0
	while i != len(path)-1:
		new_angle = angle(path[i], path[i+1])
		#new_angle *= 180/pi

		rot_angle = new_angle - old_angle
		old_angle = new_angle

		distance = math.hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y)
		vgraph.rotate(rot_angle)
		vgraph.translate(distance)

		i += 1




if __name__ == "__main__":

	bot = [[18, 18], [18, -18], [-18, 18], [-18, -18]]

	obstacles = load_obstacles("../data/world_obstacles.txt")
	
	vgraph = VGraph()
	grown_obstacles = vgraph.grow_obstacle(obstacles, bot)

	start = [0, 0]
	goal = load_goal("../data/goal.txt")

	matrix, vertices = vgraph.create_graph(start, goal, grown_obstacles)

	path = dijkstra(matrix, vertices, 0, len(matrix) - 1)

	shortest_path = vgraph.backtrack(path, vertices)

	follow_path(shortest_path)

	vgraph.draw_marker()
