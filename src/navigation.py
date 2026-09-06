import math
from utilities import dot

PATH_GAIN = -0.4
MAX_ANGLE = 35

class Navigation:
    def __init__(self, devices):
        self.drive = devices["drive"]

    def drive_path(self, path, pose, speed, debug=False):
        target_dir = path[3]
        robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
        
        err = dot(path[5], robot_vec) # how far off the robot is (in mm)
        corr = err * PATH_GAIN 
        
        # Limit correction
        if corr > MAX_ANGLE:
            corr = MAX_ANGLE
        elif corr < -MAX_ANGLE:
            corr = -MAX_ANGLE

        target_dir += corr 

        if debug:
            print("path: ", path)
            print("err, corr, target: ", err, corr, target_dir)

        self.drive.steer_p(target_dir, pose[2], speed, debug=debug)
        robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
        dist_travelled_along_path = dot(path[4], robot_vec)

        # print("target, corr directions: ", target_dir, corr)
        
        if dist_travelled_along_path >= path[2]:
            return True
        else:
            return False

    def drive_path_back(self, path, pose, speed, debug=False):
        target_dir = path[3] 
        robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
        
        err = dot(path[5], robot_vec) # how far off the robot is (in mm)
        # corr = err * PATH_GAIN
        corr = 0 
        # Limit correction
        if corr > MAX_ANGLE:
            corr = MAX_ANGLE
        elif corr < -MAX_ANGLE:
            corr = -MAX_ANGLE

        target_dir -= corr 

        if debug:
            print("path: ", path)
            print("err, corr, target: ", err, corr, target_dir)

        # print(f"Target direction: {target_dir}, gyro: {pose}")
        self.drive.steer_p_back(target_dir, pose[2], speed)

    def drive_paths(self, idx, paths, pose, speed, debug=False): # idx -- references path currently following 
        path = paths[idx % len(paths)]
        if self.drive_path(path, pose, speed, debug=debug):
            return True, idx + 1
        
        return False, idx

    def stop(self):
        self.drive.drive(0)
        self.drive.steering(0)