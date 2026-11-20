import cv2
from picamera2 import Picamera2

# Green thresholds
G_MIN_H = 44
G_MAX_H = 75

G_MIN_S = 84
G_MAX_S = 255

G_MIN_V = 41
G_MAX_V = 255

# Red thresholds
R_MIN_H = 112
R_MAX_H = 128

R_MIN_S = 90
R_MAX_S = 255

R_MIN_V = 93
R_MAX_V = 255

params = cv2.SimpleBlobDetector_Params()
params.filterByColor = True
params.blobColor = 255 # Detect white blobs. If set to 0, it'll detect black blobs.
params.filterByArea = True
params.minArea = 650
params.maxArea = 100000
params.filterByCircularity = True
params.minCircularity = 0.1
params.filterByConvexity = False
params.minConvexity = 0.87
params.filterByInertia = False
params.minInertiaRatio = 0.01

detector = cv2.SimpleBlobDetector_create(params)

# Width-to-height ratio range for the standard 100mm traffic block
# need to tune
ORIG_MIN_RATIO = 0.35
ORIG_MAX_RATIO = 0.85

picam2 = Picamera2() # resolution: 640x480
picam2.start()


class Rpicam:
    def __init__(self):
        pass

    def detect_blob(self):
        frame = picam2.capture_array("main")
        if frame is None:
            return None

        frame = frame[105:-1,0:-1] # y, x
        frame = cv2.GaussianBlur(frame, (5,5), 0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        g_frame = cv2.inRange(hsv_frame, (G_MIN_H, G_MIN_S, G_MIN_V), (G_MAX_H, G_MAX_S, G_MAX_V))
        keypoints = detector.detect(g_frame) # Detects keypoints. Each keypoint will contain the x,y position, and size.
        largest_size = 0
        g_largest_keypoint = None
        for keypoint in keypoints:
            if keypoint.size > largest_size:
                largest_size = keypoint.size
                g_largest_keypoint = keypoint
        
        r_frame = cv2.inRange(hsv_frame, (R_MIN_H, R_MIN_S, R_MIN_V), (R_MAX_H, R_MAX_S, R_MAX_V))
        keypoints = detector.detect(r_frame) # Detects keypoints. Each keypoint will contain the x,y position, and size.
        largest_size = 0
        r_largest_keypoint = None
        for keypoint in keypoints:
            if keypoint.size > largest_size:
                largest_size = keypoint.size
                r_largest_keypoint = keypoint

        # frame = cv2.drawKeypoints(frame, keypoints, 0, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow('result', frame) # Display the image in a window, for testing
        

        if g_largest_keypoint is None and r_largest_keypoint is None:
            return None
        elif g_largest_keypoint is None:
            selected_frame = r_frame
            colour = "r"
        elif r_largest_keypoint is None:
            selected_frame = g_frame
            colour = "g"
        else:
            if g_largest_keypoint.size > r_largest_keypoint.size:
                selected_frame = g_frame
                colour = "g"
            else:
                selected_frame = r_frame
                colour = "r"

        contours, _ = cv2.findContours(selected_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        _, _, width, height = cv2.boundingRect(largest_contour)
        if height == 0:
            return None

        ratio = width / height
        if ratio < ORIG_MIN_RATIO or ratio > ORIG_MAX_RATIO:
            block_type = "modified"
        else:
            block_type = "orig"

        return colour, block_type
