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

open_first_path = augment_path(open_first_path)

cw_paths = augment_paths(cw_paths)
ccw_paths = augment_paths(ccw_paths)

