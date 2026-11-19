import cv2
from picamera2 import Picamera2


# HSV thresholds
g_low_H, g_high_H = 44, 75
g_low_S, g_high_S = 84, 255
g_low_V, g_high_V = 41, 255

r_low_H, r_high_H = 112, 128
r_low_S, r_high_S = 90, 255
r_low_V, r_high_V = 93, 255

MIN_AREA = 650
MAX_AREA = 100000


def make_detector(min_area=MIN_AREA, max_area=MAX_AREA):
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


def prepare_frame(frame):
    # Apply the crop + gaussian blur
    if frame is None:
        return None

    frame = frame[105:-1, 0:-1]
    if len(frame.shape) == 3 and frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return cv2.GaussianBlur(frame, (5, 5), 0)


def largest_keypoint(detector, mask):
    # Return the largest detected blob in a binary mask
    keypoints = detector.detect(mask)
    return max(keypoints, key=lambda keypoint: keypoint.size, default=None)


class Rpicam:

    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.start()
        self.detector = make_detector()

    def detect_blob(self):
        frame = prepare_frame(self.picam2.capture_array("main"))
        if frame is None:
            return None

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(
            hsv_frame,
            (g_low_H, g_low_S, g_low_V),
            (g_high_H, g_high_S, g_high_V),
        )
        red_mask = cv2.inRange(
            hsv_frame,
            (r_low_H, r_low_S, r_low_V),
            (r_high_H, r_high_S, r_high_V),
        )

        green_keypoint = largest_keypoint(self.detector, green_mask)
        red_keypoint = largest_keypoint(self.detector, red_mask)

        if green_keypoint is None and red_keypoint is None:
            return None
        if green_keypoint is None:
            return "r"
        if red_keypoint is None:
            return "g"
        return "g" if green_keypoint.size > red_keypoint.size else "r"


if __name__ == "__main__":
    WINDOW = "Blob Detection"
    HSV_WINDOW = "HSV Controls"

    cv2.namedWindow(WINDOW)
    cv2.namedWindow(HSV_WINDOW)
    cv2.createTrackbar("Min area", WINDOW, MIN_AREA, 99999, lambda value: None)
    cv2.createTrackbar("Max area", WINDOW, MAX_AREA, 100000, lambda value: None)

    for color, values in (
        ("Green", (g_low_H, g_high_H, g_low_S, g_high_S, g_low_V, g_high_V)),
        ("Red", (r_low_H, r_high_H, r_low_S, r_high_S, r_low_V, r_high_V)),
    ):
        low_h, high_h, low_s, high_s, low_v, high_v = values
        for name, value, maximum in (
            ("low H", low_h, 179),
            ("high H", high_h, 179),
            ("low S", low_s, 255),
            ("high S", high_s, 255),
            ("low V", low_v, 255),
            ("high V", high_v, 255),
        ):
            cv2.createTrackbar(
                f"{color} {name}", HSV_WINDOW, value, maximum, lambda v: None
            )

    picam2 = Picamera2()
    picam2.start()

    try:
        while True:
            frame = prepare_frame(picam2.capture_array("main"))
            if frame is None:
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            def hsv_bounds(color):
                low = tuple(
                    cv2.getTrackbarPos(f"{color} low {channel}", HSV_WINDOW)
                    for channel in ("H", "S", "V")
                )
                high = tuple(
                    cv2.getTrackbarPos(f"{color} high {channel}", HSV_WINDOW)
                    for channel in ("H", "S", "V")
                )
                return tuple(min(value, upper) for value, upper in zip(low, high)), high

            green_low, green_high = hsv_bounds("Green")
            red_low, red_high = hsv_bounds("Red")
            green_mask = cv2.inRange(hsv_frame, green_low, green_high)
            red_mask = cv2.inRange(hsv_frame, red_low, red_high)

            max_area = max(2, cv2.getTrackbarPos("Max area", WINDOW))
            min_area = max(1, cv2.getTrackbarPos("Min area", WINDOW))
            min_area = min(min_area, max_area - 1)
            detector = make_detector(min_area, max_area)
            green_keypoint = largest_keypoint(detector, green_mask)
            red_keypoint = largest_keypoint(detector, red_mask)

            detections = []
            if green_keypoint is not None:
                detections.append(("Green", green_keypoint, (0, 255, 0)))
            if red_keypoint is not None:
                detections.append(("Red", red_keypoint, (0, 0, 255)))

            display = frame.copy()
            if detections:
                color, keypoint, draw_color = max(
                    detections, key=lambda item: item[1].size
                )
                cv2.drawKeypoints(
                    display,
                    [keypoint],
                    None,
                    draw_color,
                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
                )
                cv2.putText(
                    display,
                    f"{color} blob detected | diameter: {keypoint.size:.1f}px",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    draw_color,
                    2,
                )
                print(
                    f"{color} blob detected; diameter={keypoint.size:.1f}px; "
                    f"area range={min_area}-{max_area}px^2"
                )
            else:
                cv2.putText(
                    display,
                    "No blob detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

            cv2.imshow(WINDOW, display)
            cv2.imshow("Green HSV mask", green_mask)
            cv2.imshow("Red HSV mask", red_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
