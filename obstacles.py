cw_obstacle_positions = [
    [500, 2000],
    [1000, 2500],
    [1500, 2500]
]

def full_obstacles_from_one_section(obstacle_section):
    full_obstacles = obstacle_section
    for a in range(len(obstacle_section)):
        p = obstacle_section[a]
        full_obstacles.append([p[1], 3000-p[0]])
    for a in range(len(obstacle_section)):
        p = obstacle_section[a]
        full_obstacles.append([3000-p[0], 3000-p[1]])
    for a in range(len(obstacle_section)):
        p = obstacle_section[a]
        full_obstacles.append([3000-p[1], p[0]])
    return full_obstacles

cw_obstacle_positions = full_obstacles_from_one_section(cw_obstacle_positions)
