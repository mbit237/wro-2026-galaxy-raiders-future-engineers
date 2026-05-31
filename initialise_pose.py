from coind4 import extract_distance

def get_x(devices):
    while True:
        left_dist = devices["lidar"].get_distance(90)
        right_dist = devices["lidar"].get_distance(270)
        
        # Wall extended
        if 500 < left_dist + right_dist < 700: 
            return ((600 - right_dist) + left_dist) / 2
        
        # Wall flushed
        elif 900 < left_dist + right_dist < 1100: 
            return ((1000 - right_dist) + left_dist) / 2

def get_y(devices):
    while True:
        fwd_dist = devices["lidar"].get_distance(0)
        rear_dist = devices["lidar"].get_distance(180)

        # both readings valid
        if 2900 < fwd_dist + rear_dist < 3100: 
            return ((3000 - fwd_dist) + rear_dist) / 2
        
        # front reading valid
        elif 1000 < fwd_dist < 2000:
            return 3000 - fwd_dist
        
        # rear reading valid
        elif 1000 < rear_dist < 2000:
            return rear_dist

def open(devices):
    x = get_x(devices)
    y = get_y(devices)
    return [x, y, 90]

def confirm_pose(pose, sensor_readings):
    angle_from_north = pose[2] - 90 
    left_angle = 40 - angle_from_north
    right_angle = 320 - angle_from_north
    left_dist = extract_distance(sensor_readings["lidar"], left_angle)
    right_dist = extract_distance(sensor_readings["lidar"], right_angle)
    if left_dist > right_dist:
        pose[0] = 3000 - pose[0]
    return pose

# Challenges
# - Fwd and back dist were invalid because they were too far away 
#   - only look at one of the readings, discard the invalid 
#   - if both are invalid, keep reading it until it returns at least one valid reading
# - Hard to determine left or right, especially when it is close to the inner wall
#   so it is unable to detect spikes on inner corners 
#   - Defer decision: Assume it is on the left or right 
#   - Move forward to determine if it is on the left or right
