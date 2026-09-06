cw_obstacle_positions = [
    [500, 2000],
    [1000, 2500],
    [1000, 2500],
    [1500, 2500]
]

ccw_obstacle_positions = [
    [2500, 2000],
    [2500, 1500],
    [2500, 1000],
    [2500, 1000]
]

def full_obstacles_from_one_section(obstacle_section):
    full_obstacles = obstacle_section
    obstacle_len = len(obstacle_section)
    for a in range(obstacle_len):
        p = obstacle_section[a]
        full_obstacles.append([p[1], 3000-p[0]])
    for a in range(obstacle_len):
        p = obstacle_section[a]
        full_obstacles.append([3000-p[0], 3000-p[1]])
    for a in range(obstacle_len):
        p = obstacle_section[a]
        full_obstacles.append([3000-p[1], p[0]])
    return full_obstacles

cw_obstacle_positions = full_obstacles_from_one_section(cw_obstacle_positions)
# print(cw_obstacle_positions, len(cw_obstacle_positions))

ccw_obstacle_positions = full_obstacles_from_one_section(ccw_obstacle_positions)
ccw_obstacle_positions = ccw_obstacle_positions[::-1]
# print(ccw_obstacle_positions, len(ccw_obstacle_positions))
