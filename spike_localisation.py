import math

POSITION_FILTER_RATIO = 0.1 
HEADING_FILTER_RATIO = 0.01 

# LANDMARKS = [
#     [1000, 1000],   #first 4 landmarks are outer landmarks
#     [1000, 2000],
#     [2000, 1000],
#     [2000, 2000],
#     [0, 0],        #last 4: inner landmarks
#     [0, 3000],
#     [3000, 0],
#     [3000, 3000]
# ]
LANDMARKS = [ # open challenge
    [1000, 1000],   #first 4 landmarks are outer landmarks
    [1000, 2000],
    [2000, 1000],
    [2000, 2000],
    [1000, 600],    # additional landmarks for open challenge
    [600, 600],    
    [600, 1000],    
    [600, 2000],
    [600, 2400],
    [1000, 2400],
    [2000, 2400],
    [2400, 2400],
    [2400, 2000],
    [2400, 1000],
    [2400, 600],
    [2000, 600],
    [0, 0],        #next 4: inner landmarks
    [0, 3000],
    [3000, 0],
    [3000, 3000],
]

LANDMARKS = [ # obstacle challenge
    [1000, 1000],   #first 4 landmarks are outer landmarks
    [1000, 2000],
    [2000, 1000],
    [2000, 2000],
    [0, 0],        #last 4: inner landmarks
    [0, 3000],
    [3000, 0],
    [3000, 3000],
    # additional landmarks for obstacle challenge
    [400, 1000],  # left side
    [600, 1000],   
    [400, 1500],
    [600, 1500],
    [400, 2000],
    [600, 2000],
    [1000, 2600], # top side
    [1000, 2400],
    [1500, 2600],
    [1500, 2400],
    [2000, 2600],
    [2000, 2400],
    [2400, 2000], # right side
    [2600, 2000],
    [2400, 1500],
    [2600, 1500],
    [2400, 1000],
    [2600, 1000],
    [2000, 600],  # bottom side
    [2000, 400],
    [1500, 600],
    [1500, 400],
    [1000, 600],
    [1000, 400]
]

LANDMARK_THRESHOLD = 120
M_THRESHOLD = 100

def identify_spikes(measurements):
    spikes = []
    for d in range(len(measurements)):
        prev_d = measurements[d-1][1]
        curr_d = measurements[d][1]
        next_d = measurements[(d+1) % len(measurements)][1] 
        if prev_d < 250 or next_d < 250 or curr_d < 250:  # if previous or next distance is too small, skip this measurement
            continue
        if abs((prev_d - curr_d) + (next_d - curr_d)) > M_THRESHOLD:
            if 250 < measurements[d][1] < 1500:
                spikes.append(measurements[d])  
            
    return spikes

def add_cartesian(pose, spikes):# spikes in cartesian coordinates
    c_spikes = []
    for spike in spikes:
        distance = spike[1]
        theta = (pose[2] + spike[0]) * math.pi / 180 # in radians, robot angle + lidar's spike angle
        dx = distance * math.cos(theta)
        dy = distance * math.sin(theta)
    
        x = pose[0] + dx
        y = pose[1] + dy
        c_spikes.append([x, y, spike[0], distance])
        
    return c_spikes

def match_landmarks(c_spikes):
    matches = []
    for cs in c_spikes:
        for landmark in LANDMARKS:
            dx = cs[0] - landmark[0]
            dy = cs[1] - landmark[1]
            dist = math.sqrt(dx**2 + dy**2)
            err_x = round(abs(cs[0] - landmark[0]), 2)
            err_y = round(abs(cs[1] - landmark[1]), 2)
            if dist < LANDMARK_THRESHOLD:
                # matches.append([round(cs[0], 2), round(cs[1], 2), landmark[0], landmark[1], cs[2], cs[3]])
                matches.append([round(cs[0], 2), round(cs[1], 2), landmark[0], landmark[1], cs[2], cs[3], err_x, err_y])
                break
    return matches

def calc_position_error(matches):
    if not matches:  # Handle empty matches list
        return [0, 0]
    errors = []
    for match in matches:
        errorx = match[0] - match[2]
        errory = match[1] - match[3]
        errors.append([errorx, errory])
        
    err_x = 0
    err_y = 0
    for err in errors:
        err_x += err[0]
        err_y += err[1]
    ave_err_x = err_x / len(errors)
    ave_err_y = err_y / len(errors)
    
    return [ave_err_x, ave_err_y]

def calc_pose(pose, error):
    new_x = pose[0] - error[0]
    new_y = pose[1] - error[1]
    
    return [new_x, new_y, pose[2]]

def merge_positions(odo_pose, spike_pose, POSITION_FILTER_RATIO):
    odo_x = odo_pose[0]
    odo_y = odo_pose[1]
    spike_x = spike_pose[0]
    spike_y = spike_pose[1]
    merged_x = odo_x * (1 - POSITION_FILTER_RATIO) + spike_x * POSITION_FILTER_RATIO
    merged_y = odo_y * (1 - POSITION_FILTER_RATIO) + spike_y * POSITION_FILTER_RATIO
    
    return [merged_x, merged_y, odo_pose[2]]

def calc_angle_error(pose, matches):
    if not matches:
        return 0
    
    angle_errors = []
    
    for match in matches:
        robot_x = pose[0]
        robot_y = pose[1]
        landmark_x = match[2]
        landmark_y = match[3]
        dx = landmark_x - robot_x
        dy = landmark_y - robot_y
        a_theta = math.atan2(dy, dx) * 180 / math.pi # actual theta
        m_theta = pose[2] + match[4] # measured_theta
        angle_err = m_theta - a_theta
        while angle_err > 180:
            angle_err -= 360
        while angle_err < -180:
            angle_err += 360
        
        angle_errors.append(angle_err)
    # print(angle_errors)
    ave_angle_err = sum(angle_errors) / len(angle_errors)
    return ave_angle_err

def calc_heading(pose, error):
    return [pose[0], pose[1], (pose[2] - error)]

def localise(odometry_pose, sensor_readings):
    if not sensor_readings["lidar"]:
        return False
    
    spikes = identify_spikes(sensor_readings["lidar"])
    c_spikes = add_cartesian(odometry_pose, spikes)
    matches = match_landmarks(c_spikes)

    while odometry_pose[2] > 180:
        odometry_pose[2] -= 360
    while odometry_pose[2] < -180:
        odometry_pose[2] += 360

    position_error = calc_position_error(matches)
    spike_pose = calc_pose(odometry_pose, position_error)
    merged_position_pose = merge_positions(odometry_pose, spike_pose, POSITION_FILTER_RATIO)
    angle_error = calc_angle_error(merged_position_pose, matches)
    spike_heading_pose = calc_heading(merged_position_pose, angle_error)

    return [spike_pose[0], spike_pose[1], spike_heading_pose[2]]
    
    