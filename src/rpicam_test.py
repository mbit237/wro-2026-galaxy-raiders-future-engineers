import cv2

# Default
g_low_H, g_high_H = 44, 75
g_low_S, g_high_S = 84, 255
g_low_V, g_high_V = 41, 255

r_low_H, r_high_H = 112, 128
r_low_S, r_high_S = 90, 255
r_low_V, r_high_V = 93, 255

WINDOW = "Blob Detection"
HSV_WINDOW = "HSV Controls"
MIN_AREA = 650
MAX_AREA = 100000


def make_detector(min_area, max_area):
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = min_area
    params.maxArea = max_area
    params.filterByCircularity = True
    params.minCircularity = 0.1
    params.filterByConvexity = False
    params.minConvexity = 0.87
    params.filterByInertia = False
    params.minInertiaRatio = 0.01
    return cv2.SimpleBlobDetector_create(params)


camera = cv2.VideoCapture(0)
if not camera.isOpened():
    raise RuntimeError("Could not open webcam 0")
cv2.namedWindow(WINDOW)
cv2.namedWindow(HSV_WINDOW)
cv2.createTrackbar("Min area", WINDOW, MIN_AREA, 99999,
                   lambda value: None)
cv2.createTrackbar("Max area", WINDOW, MAX_AREA, 100000,
                   lambda value: None)

for color, values in (("Green", (g_low_H, g_high_H, g_low_S, g_high_S,
                                  g_low_V, g_high_V)),
                      ("Red", (r_low_H, r_high_H, r_low_S, r_high_S,
                               r_low_V, r_high_V))):
    low_h, high_h, low_s, high_s, low_v, high_v = values
    cv2.createTrackbar(f"{color} low H", HSV_WINDOW, low_h, 179, lambda v: None)
    cv2.createTrackbar(f"{color} high H", HSV_WINDOW, high_h, 179, lambda v: None)
    cv2.createTrackbar(f"{color} low S", HSV_WINDOW, low_s, 255, lambda v: None)
    cv2.createTrackbar(f"{color} high S", HSV_WINDOW, high_s, 255, lambda v: None)
    cv2.createTrackbar(f"{color} low V", HSV_WINDOW, low_v, 255, lambda v: None)
    cv2.createTrackbar(f"{color} high V", HSV_WINDOW, high_v, 255, lambda v: None)

try:
    while True:
        success, frame = camera.read()
        if not success or frame is None:
            break

        frame = frame[105:-1, 0:-1]
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        def hsv_bounds(color):
            low = tuple(cv2.getTrackbarPos(f"{color} low {channel}", HSV_WINDOW)
                        for channel in ("H", "S", "V"))
            high = tuple(cv2.getTrackbarPos(f"{color} high {channel}", HSV_WINDOW)
                         for channel in ("H", "S", "V"))
            # Keep every lower bound below its upper bound, even while tuning.
            low = tuple(min(value, upper) for value, upper in zip(low, high))
            return low, high

        green_low, green_high = hsv_bounds("Green")
        red_low, red_high = hsv_bounds("Red")

        green_mask = cv2.inRange(
            hsv_frame, green_low, green_high)
        red_mask = cv2.inRange(
            hsv_frame, red_low, red_high)

        min_area = cv2.getTrackbarPos("Min area", WINDOW)
        max_area = cv2.getTrackbarPos("Max area", WINDOW)
        max_area = max(2, max_area)
        min_area = max(1, min(min_area, max_area - 1))
        detector = make_detector(min_area, max_area)

        green_keypoints = detector.detect(green_mask)
        red_keypoints = detector.detect(red_mask)
        detections = [("Green", kp, (0, 255, 0)) for kp in green_keypoints]
        detections += [("Red", kp, (0, 0, 255)) for kp in red_keypoints]

        display = frame.copy()
        if detections:
            color, keypoint, draw_color = max(
                detections, key=lambda item: item[1].size)
            cv2.drawKeypoints(
                display, [keypoint], None, draw_color,
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.putText(
                display,
                f"{color} blob detected | diameter: {keypoint.size:.1f}px",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2)
            print(f"{color} blob detected; diameter={keypoint.size:.1f}px; "
                  f"area range={min_area}-{max_area}px^2")
        else:
            cv2.putText(display, "No blob detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow(WINDOW, display)
        cv2.imshow("Green HSV mask", green_mask)
        cv2.imshow("Red HSV mask", red_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
finally:
    camera.release()
    cv2.destroyAllWindows()


# Green 
# 100mm 
# 45 -- 26952-28369 -- 27600
# 60 cm -- 16818-18814 -- 17800
# 90 -- 6564-7067 -- 6800
# 100 -- 5096-5873 -- 3000