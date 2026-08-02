import math 

def dir_to_point(pose, point):
    # given a pose and x, y coordinate, return a direction relative to the robot, ie. 0 is straight ahead
    angle_uncorrected = math.degrees(math.atan2(point[1] - pose[1], point[0] - pose[0]))
    angle = angle_uncorrected - pose[2]
    while angle < 0:
        angle += 360
    while angle > 180:
        angle -= 360
    return angle

def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]
