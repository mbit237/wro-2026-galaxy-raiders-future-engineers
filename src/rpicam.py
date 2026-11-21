import cv2
from picamera2 import Picamera2
import numpy as np

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

SMALL_BLOCK_RATIO = 0.3
LARGE_BLOCK_RATIO = 1.0

picam2 = Picamera2() # resolution: 640x480
picam2.start()


class Rpicam:
    def __init__(self):
        pass

    def detect_blob(self):
        frame = picam2.capture_array("main")
        if frame is None:
            return None

        orig_frame = frame
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

        largest_keypoint = None
        if g_largest_keypoint is None and r_largest_keypoint is None:
            return None
        elif g_largest_keypoint is None:
            colour = "r"
            largest_keypoint = r_largest_keypoint
        elif r_largest_keypoint is None:
            colour = "g"
            largest_keypoint = g_largest_keypoint
        else:
            if g_largest_keypoint.size > r_largest_keypoint.size:
                colour = "g"
                largest_keypoint = g_largest_keypoint
            else:
                colour = "r"
                largest_keypoint = r_largest_keypoint

        block_type = self.identify_block_type(colour, orig_frame, largest_keypoint.pt, largest_keypoint.size)

        return colour, block_type
    
    def find_largest_contour(self, mask, min_area):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [contour for contour in contours if cv2.contourArea(contour) >= min_area]
        return max(candidates, key=cv2.contourArea, default=None)

    def identify_block_type(self, colour, frame, coords, size):
        # coords of centre (x, y)
        # size - diameter
        if colour is None:
            return None

        frame = cv2.GaussianBlur(frame, (5,5), 0) # Gaussian Blur
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # BGR -> HSV 
        # frame = frame[60:-1,0:-1] - Less Cropping

        # Slicing 
        x = coords[0]
        # y = coords[1]

        sliced_frame = frame[0:-1, x-size//2:x+size//2]
        
        mask = None 
        if colour == "g":
            mask = cv2.inRange(sliced_frame, (G_MIN_H, G_MIN_S, G_MIN_V), (G_MAX_H, G_MAX_S, G_MAX_V))
        elif colour == "r":
            mask = cv2.inRange(sliced_frame, (R_MIN_H, R_MIN_S, R_MIN_V), (R_MAX_H, R_MAX_S, R_MAX_V))

        # Drawing mask
        cv2.imshow(f"{colour} mask", mask)
        
        contour = self.find_largest_contour(mask, params.min_area)
        if contour is None:
            return None
        
        x, y, width, height = cv2.boundingRect(contour)

        print("First bounding box: ", [x, y, width, height])

        # Drawing contour
        display = frame.copy()
        draw_colour = (0, 255, 0) if colour == "Green" else (0, 0, 255)
        cv2.drawContours(display, [contour], -1, draw_colour, 2)
        cv2.rectangle(display, (x, y), (x + width, y + height), draw_colour, 2)
    
        bottom_y = y + height
        bottom_slice_height = 20 
        bottom_slice = mask[bottom_y-bottom_slice_height:bottom_y, x:x+width]
        bottom_contour = self.find_largest_contour(bottom_slice, -1)
        b_x, b_y, b_width, b_height = cv2.boundingRect(bottom_contour)
        
        print("bottom slice: ", b_x, b_y, b_width, b_height)

        left_right_slice_width = 10 
        left_slice = mask[y:y+height, x:x+left_right_slice_width]
        right_slice = mask[y:y+height, x+width-left_right_slice_width:x+width]
        left_contour = self.find_largest_contour(left_slice, -1)
        right_contour = self.find_largest_contour(right_slice, -1)
        l_x, l_y, l_width, l_height = cv2.boundingRect(left_contour)
        # print("left_height: ", l_height, height)
        r_x, r_y, r_width, r_height = cv2.boundingRect(right_contour)
        # print("right_height: ", r_height, height)
        
        print("left_slice: ", l_x, l_y, l_width, l_height)
        print("right_slice: ", r_x, r_y, r_width, r_height)
        
        height = 0 
        if l_y > r_y:
            # print('left')
            height = l_height 
        else:
            # print('right')
            height = r_height
        # print('height', height)

        ratio = b_width/height if height else 0.0
        # print('true aspect ratio', ratio)

        if ratio <= SMALL_BLOCK_RATIO or ratio >= LARGE_BLOCK_RATIO:
            block_type = "modified"
        else:
            block_type = "orig"

        return block_type

# Debugging
rpicam = Rpicam()
colour, blocktype = rpicam.detect_blob()
print("Colour: ", colour)
print("Blocktype: ", blocktype)