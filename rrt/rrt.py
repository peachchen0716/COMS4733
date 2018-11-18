# Program to load obstacle course for Lab 4 - RRT

# usage:  python rrt.py obstacles_file start_goal_file


from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math
import rrtUtil as RU
import time

MAX_NUM_VERT = 1500
STEP_DISTANCE = 15

class config:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.tup())

    def __str__(self):
        return str(self.tup())

    def tup(self):
        return (self.x, self.y)

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

def rand(dim_x, dim_y):
    # 600 by 600 map
    # Obstacle detection not implemented yet
    random.seed()
    return config(random.randint(0,dim_x), random.randint(0,dim_y))

def near(q_rand, G):
    def dist(q1, q2):
        return math.sqrt((q2.y - q1.y)**2 + (q2.x - q1.x)**2)

    min_dist = np.inf
    q = config(0,0)
    for point in G:
        vertex = config(point[0], point[1])
        if dist(vertex, q_rand) < min_dist:
            min_dist = dist(vertex, q_rand)
            q = vertex

    return q

def new(q_near, q_rand, dq):
    def angle(q1, q2):
        return math.atan2(q_rand.y-q_near.y, q_rand.x-q_near.x)

    theta = angle(q_near, q_rand)
    new_x = dq * np.cos(theta) + q_near.x
    new_y = dq * np.sin(theta) + q_near.y

    return config(new_x, new_y)

def tuplefy(q):
    return (q.x, q.y)

def printc(q):
    print(q.x, q.y)


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)
    obs = RU.build_obs_list(args.obstacle_path)

    q_start = config(start[0], start[1]) #otherwise, start
    G = {tuplefy(q_start):[]}
    P = {}
    q_goal = config(goal[0], goal[1])
    goal_found = False
    k = 0

    # bias values (5 steps around the goal point)
    minX = max(goal[0] - 5 * STEP_DISTANCE, 0)
    maxX = min(goal[0] + 5 * STEP_DISTANCE, 600)
    minY = max(goal[1] - 5 * STEP_DISTANCE, 0)
    maxY = min(goal[1] + 5 * STEP_DISTANCE, 600)

    while not goal_found and len(G) < MAX_NUM_VERT:

        if (k % 200) == 0 and k > 0:
            print("%d vertex generated" % (len(G)))

        while True:
            if random.random() > 0.95:
                # q_rand = config(random.randint(minX, maxX), random.randint(minY, maxY))
                q_rand = q_goal
            else:
                q_rand = rand(600, 600)
            if not RU.is_inside(obs, q_rand):
                break

        k += 1
        q_near = near(q_rand, G)
        q_new = new(q_near, q_rand, STEP_DISTANCE)
        if RU.is_invalid(obs, q_new, q_near):
            continue

        G[tuplefy(q_new)] = []
        G[tuplefy(q_near)].append(tuplefy(q_new))
        P[q_new] = q_near

        ax.plot(q_new.x, q_new.y, 'bo', markersize=1)
        ax.plot([ q_near.x, q_new.x ], [ q_near.y, q_new.y ], color='blue', linestyle='-', linewidth=0.5)

        if abs(q_goal.x - q_new.x) < 5 and abs(q_goal.y - q_new.y) < 5:
            print("FOUND GOAL")
            # draw shortest path
            parent = P[q_new]
            while parent != q_start:
                ax.plot([ parent.x, q_new.x ], [ parent.y, q_new.y ], color='yellow', linestyle='-', linewidth=2)
                q_new = parent
                parent = P[q_new]
            plt.draw()
            goal_found = True
        plt.draw()

    plt.show()


    