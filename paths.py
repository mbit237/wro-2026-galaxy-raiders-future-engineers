import math

OPEN_FIRST_PATH = [[300, 300], [300, 2400]]

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

OPEN_FIRST_PATH = augment_path(OPEN_FIRST_PATH)

