import math 
import numpy as np

from utilities import dot

PERPENDICULAR_DIST_THRESHOLD = 100

walls = [
    # outer walls, clockwise dir
    [[0, 0], [0, 3000]], 
    [[0, 3000], [3000, 3000]], 
    [[3000, 3000], [3000, 0]], 
    [[3000, 0], [0, 0]], 

    # inner walls, clockwise dir
    [[1000, 1000], [1000, 2000]], 
    [[1000, 2000], [2000, 2000]], 
    [[2000, 2000], [2000, 1000]], 
    [[2000, 1000], [1000, 1000]]
]

def augment_wall(wall):
    dx = wall[1][0] - wall[0][0]
    dy = wall[1][1] - wall[0][1]
    
    dist = math.sqrt(dx**2 + dy**2)
    path_vec = [dx, dy]
    path_dir = math.atan2(dy, dx) * 180 / math.pi # in degrees
    unit_path_vec = [path_vec[0] / dist, path_vec[1] / dist]
    p_unit_path_vec = [-1 * unit_path_vec[1], unit_path_vec[0]] 
    
    wall.append(path_vec) # 2
    wall.append(path_dir) # 3
    wall.append(unit_path_vec) # 4 
    wall.append(p_unit_path_vec) # 5
    wall.append(dist) # 6
    return wall 

def augment_walls(walls):
    for w in range(len(walls)):
        walls[w] = augment_wall(walls[w])
    return walls

walls = augment_walls(walls)

def add_cartesian(pose, lidar_readings):# lidar_readings in cartesian coordinates
    c_lidar_readings = []
    for lidar_reading in lidar_readings:
        distance = lidar_reading[1]
        theta = (pose[2] + lidar_reading[0]) * math.pi / 180 # in radians, robot angle + lidar's spike angle
        dx = distance * math.cos(theta)
        dy = distance * math.sin(theta)
    
        x = pose[0] + dx
        y = pose[1] + dy
        c_lidar_readings.append([x, y, lidar_reading[0], distance])
        
    return c_lidar_readings

def calc_pose(Tx, Ty, theta, odometry_pose):
    new_x = odometry_pose[0] + Tx
    new_y = odometry_pose[1] + Ty
    new_angle = odometry_pose[2] + theta

    return [new_x, new_y, new_angle]

def localise(odometry_pose, sensor_readings):
    if not sensor_readings["lidar"]:
        return False
    
    matrix_A = []
    matrix_B = []
    
    c_lidar_readings = add_cartesian(odometry_pose, sensor_readings["lidar"])

    for c_lidar_reading in c_lidar_readings:
        for wall in walls:
            x1 = c_lidar_reading[0]
            y1 = c_lidar_reading[1]

            lidar_reading_from_wall_start_vec = [c_lidar_reading[0] - wall[0][0], c_lidar_reading[1] - wall[0][1]]
            perpendicular_dist_from_wall = dot(wall[5], lidar_reading_from_wall_start_vec) # p_unit_vec * vec_start_from_lidar_point
            distance_from_wall_start = dot(wall[4], lidar_reading_from_wall_start_vec)

            # Found match
            if abs(perpendicular_dist_from_wall) <= PERPENDICULAR_DIST_THRESHOLD and 0 <= distance_from_wall_start <= wall[6]:
                if wall[3] == 90 or wall[3] == 270: # wall direction vertical
                    matrix_A.append([-y1, 1, 0])
                    matrix_B.append([wall[0][0] - x1])

                elif wall[3] == 0 or wall[3] == 180: # wall direction horizontal
                    matrix_A.append([x1, 0, 1])
                    matrix_B.append([wall[0][1] - y1])

                break 

    matrix_A = np.matrix(matrix_A)
    matrix_B = np.matrix(matrix_B)

    psuedoinverse_A = np.linalg.pinv(matrix_A)
    matrix_x = np.matmul(psuedoinverse_A, matrix_B)
    
    Tx = float(matrix_x[1][0])
    Ty = float(matrix_x[2][0])
    theta = float(matrix_x[0][0] / math.pi * 180)
    point_cloud_pose = calc_pose(Tx, Ty, theta, odometry_pose)

    return point_cloud_pose

def localise_iter(odometry_pose, sensor_readings, iter=2):
    curr_iter = 0 
    curr_init_pose = odometry_pose
    while curr_iter < iter:
        curr_init_pose = localise(curr_init_pose, sensor_readings)
        curr_iter += 1
    
    return curr_init_pose 