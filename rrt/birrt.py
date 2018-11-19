import rrtUtil as RU
import rrt as R
import random

from rrt import config
from rrt import MAX_NUM_VERT
import matplotlib.pyplot as plt

def get_rand(obs, bias):
    while True:
        if random.random() > 0.95:
            q_rand = bias
        else:
            q_rand = R.rand(600, 600)
        if not RU.is_inside(obs, q_rand):
            break
    return q_rand

def draw_path(ax, q_new, P, clr, q_end):
    parent = P[q_new]
    while q_new != q_end:
        ax.plot([ parent.x, q_new.x ], [ parent.y, q_new.y ], color=clr, linestyle='-', linewidth=2)
        q_new = parent
        parent = P[q_new]

if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    parser.add_argument('step_distance',
                        help="Tree growth distance")
    args = parser.parse_args()

    STEP_DISTANCE = int(args.step_distance)

    fig, ax = plt.subplots()
    path = R.build_obstacle_course(args.obstacle_path, ax)
    start, goal = R.add_start_and_goal(args.start_goal_path, ax)
    obs = RU.build_obs_list(args.obstacle_path)

    q_start = config(start[0], start[1])
    G_start = {q_start.tup():[]}
    P_start = {q_start: None}
    q_goal = config(goal[0], goal[1])
    G_goal = {q_goal.tup():[]}
    P_goal = {q_goal: None}

    goal_found = False
    k = 0

    while not goal_found and len(G_start) + len(G_goal) < MAX_NUM_VERT:

        if (k % 200) == 0 and k > 0:
            print("%d vertex generated" % (len(G_start) + len(G_goal)))

        k += 1
        q_rand = get_rand(obs, q_goal)
        q_near = R.near(q_rand, G_start)
        q_new = R.new(q_near, q_rand, q_goal, STEP_DISTANCE)
        if RU.is_invalid(obs, q_new, q_near):
            continue

        G_start[q_new.tup()] = []
        G_start[q_near.tup()].append(q_new.tup())
        P_start[q_new] = q_near

        ax.plot(q_new.x, q_new.y, 'bo', markersize=1)
        ax.plot([ q_near.x, q_new.x ], [ q_near.y, q_new.y ], color='blue', linestyle='-', linewidth=0.5)

        # the point closest to q_new in G_goal
        q2_near = R.near(q_new, G_goal)
        q2_new = R.new(q2_near, q_new, q_new, STEP_DISTANCE)

        if q2_new == q_new and (not RU.is_invalid(obs, q_new, q2_near)):

            # G_start[q_new.tup()] = []
            # G_start[q_near.tup()].append(q_new.tup())
            # P_start[q_new] = q_near

            P_goal[q2_new] = q2_near
            print("FOUND GOAL")
            draw_path(ax, q_new, P_start, 'yellow', q_start)
            draw_path(ax, q_new, P_goal, 'green', q_goal)

            plt.draw()
            goal_found = True
            break

        k += 1
        stop = False
        while True:
            if stop:
                break
            q_rand = get_rand(obs, q_start)
            q_near = R.near(q_rand, G_goal)
            q_new = R.new(q_near, q_rand, q_goal, STEP_DISTANCE)
            if RU.is_invalid(obs, q_new, q_near):
                continue
            stop = True
            G_goal[q_new.tup()] = []
            G_goal[q_near.tup()].append(q_new.tup())
            P_goal[q_new] = q_near

            ax.plot(q_new.x, q_new.y, 'ro', markersize=1)
            ax.plot([ q_near.x, q_new.x ], [ q_near.y, q_new.y ], color='red', linestyle='-', linewidth=0.5)
            # if q_new in G_start:
            #     print("FOUND GOAL")
            #     draw_path(ax, q_new, P_goal, 'green')
            #     draw_path(ax, q_new, P_start, 'yellow')
                    
            #     plt.draw()
            #     goal_found = True

    plt.show()