from rrt import config

def on_segment(p, q, r): 
    if q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and \
        q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y): 
        return True; 
    return False; 

def orientation(p, q, r):
    val = (q.y - p.y) * (r.x - q.x) - \
            (q.x - p.x) * (r.y - q.y)

    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2

def do_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4: 
        return True; 
    if o1 == 0 and on_segment(p1, p2, q1):
        return True; 
    if o3 == 0 and on_segment(p2, p1, q2):
        return True; 
    if o4 == 0 and on_segment(p2, q1, q2): 
        return True; 
    return False

def is_path_inside(obs, p1, p2):

    for ob in obs:
        for k in range(len(ob)):
            l = 0 if (k + 1) == len(ob) else k + 1 # edge case
            p3 = ob[k]
            p4 = ob[l]
            if do_intersect(p1, p2, p3, p4):
                return True
    
    return False

def is_inside_ob(ob, n, p):
    if n < 3:
        return False
    
    extreme = config(1000, p.y)
    count = 0
    i = 0
    
    while True:
        next_i = (i + 1) % n
        if do_intersect(ob[i], ob[next_i], p, extreme):
            if orientation(ob[i], p, ob[next_i]) == 0:
                 return on_segment(ob[i], p, ob[next_i])
            count += 1
        i = next_i
        if i == 0:
            break
    return count % 2 == 1

def is_inside(obs, p):
    for ob in obs:
        if is_inside_ob(ob, len(ob), p):
            return True
    return False

def is_invalid(obs, p, q):
    return is_inside(obs, p) or is_path_inside(obs, p, q)


def build_obs_list(obstacle_path):
    '''
        returns a list of obstacles (represented by a list of configs)
    '''
    obs = list()
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        for i in range(quantity):
            ob = list()
            n = int(f.readline())
            for j in range(n):
                line = f.readline().strip().split(' ')
                ob.append(config(int(line[0]), int(line[1])))
            obs.append(ob)
    return obs