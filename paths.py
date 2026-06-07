import math

open_first_path = [[500, 300], [500, 2400]]

cw_paths = [ 
    [[300, 300], [300, 2350]],
    [[300, 2700], [2350, 2700]],
    [[2700, 2700], [2700, 650]],
    [[2700, 300], [650, 300]]
]

ccw_paths = [ 
    [[2700, 300], [2700, 2350]],
    [[2700, 2700], [650, 2700]],
    [[300, 2700], [300, 650]],
    [[300, 300], [2350, 300]]
]

obstacle_positions = [
    
]

outer_one_section = [
    # straight path #1
    [[200, 1000], [200, 1200]],

    [[200, 1200], [200, 1350]],
    # check colour
    [[200, 1500], [200, 2000]], # [[200, 1500], [200, 1950]],

    # turning point #1
    [[200, 2000], [450, 2150.67]], # [[200, 2000], [600, 2200]],
    # check colour
    [[650, 2300], [650, 2346.7]], # [[700, 2400], [700, 2450]],
    # check colour
]

inner_one_section = [
    # straight path #1
    [[800, 1000], [800, 1200]],
    # check colour
    [[800, 1200], [800, 1350]], 
    # check colour
    #turning point #1
    [[800, 1400], [800, 2000]],
    [[800, 2200], [850, 2200]],
    [[850, 2200], [1000, 2200]], # [[850, 2200], [900, 2200]],
    # check colour
]

cw_parking_path = [[335, 1350], [335, 2000]]
ccw_parking_path = [[2400, 1600], [2900, 1800]]

def augment_path(path):
    dx = path[1][0] - path[0][0]
    dy = path[1][1] - path[0][1]
    
    dist = math.sqrt(dx**2 + dy**2)
    path_dir = math.atan2(dy, dx) * 180 / math.pi # in degrees
    path_vec = [dx, dy]
    unit_path_vec = [path_vec[0] / dist, path_vec[1] / dist]
    p_unit_path_vec = [-1 * unit_path_vec[1], unit_path_vec[0]] 
    
    path.append(dist)
    path.append(path_dir)
    path.append(unit_path_vec)
    path.append(p_unit_path_vec)
    return path 

def augment_paths(paths):
    for p in range(len(paths)):
        paths[p] = augment_path(paths[p])
    return paths

def full_path_from_one_section(outer_one_section, inner_one_section):
    obstacle_inner_paths = inner_one_section
    obstacle_outer_paths = outer_one_section
    for a in range(5):
        p = inner_one_section[a]
        obstacle_inner_paths.append([[p[0][1], 3000-p[0][0]], [p[1][1], 3000-p[1][0]]])
        p = outer_one_section[a]
        obstacle_outer_paths.append([[p[0][1], 3000-p[0][0]], [p[1][1], 3000-p[1][0]]])
    for a in range(5):
        p = inner_one_section[a]
        obstacle_inner_paths.append([[3000-p[0][0], 3000-p[0][1]], [3000-p[1][0], 3000-p[1][1]]])
        p = outer_one_section[a]
        obstacle_outer_paths.append([[3000-p[0][0], 3000-p[0][1]], [3000-p[1][0], 3000-p[1][1]]])
    for a in range(5):
        p = inner_one_section[a]
        obstacle_inner_paths.append([[3000-p[0][1], p[0][0]], [3000-p[1][1], p[1][0]]])
        p = outer_one_section[a]
        obstacle_outer_paths.append([[3000-p[0][1], p[0][0]], [3000-p[1][1], p[1][0]]])

    # avoid the parking wall when taking the outer paths
    obstacle_outer_paths[0] = [[400, 1000], [400, 1200]]
    obstacle_outer_paths[1] = [[400, 1200], [400, 1350]]
    obstacle_outer_paths[19] = [[600, 700], [559, 700]]
    return obstacle_outer_paths, obstacle_inner_paths
    
def ccw_paths_from_cw(obstacle_outer_paths, obstacle_inner_paths):
    ccw_obstacle_inner_paths = []
    ccw_obstacle_outer_paths = []
    for a in range(len(obstacle_inner_paths)):
        p = obstacle_inner_paths[a]
        ccw_obstacle_inner_paths.append([[3000-p[0][0], p[0][1]], [3000-p[1][0], p[1][1]]])
        p = obstacle_outer_paths[a]
        ccw_obstacle_outer_paths.append([[3000-p[0][0], p[0][1]], [3000-p[1][0], p[1][1]]])

    # avoid the parking wall
    ccw_obstacle_outer_paths[1] = [[2600, 1200], [2600, 1350]]
    ccw_obstacle_outer_paths[2] = [[2600, 1500], [2600, 2000]]
    return ccw_obstacle_outer_paths, ccw_obstacle_inner_paths

# Open
open_first_path = augment_path(open_first_path)

cw_paths = augment_paths(cw_paths)
ccw_paths = augment_paths(ccw_paths)

# Obstacle
cw_obstacle_outer_paths, cw_obstacle_inner_paths = full_path_from_one_section(outer_one_section, inner_one_section)
ccw_obstacle_outer_paths, ccw_obstacle_inner_paths = ccw_paths_from_cw(cw_obstacle_outer_paths, cw_obstacle_inner_paths)

cw_obstacle_outer_paths = augment_paths(cw_obstacle_outer_paths)
cw_obstacle_inner_paths = augment_paths(cw_obstacle_inner_paths)
cw_parking_path = augment_path(cw_parking_path)

ccw_obstacle_outer_paths = augment_paths(ccw_obstacle_outer_paths)
ccw_obstacle_inner_paths = augment_paths(ccw_obstacle_inner_paths)
ccw_parking_path = augment_path(ccw_parking_path)



