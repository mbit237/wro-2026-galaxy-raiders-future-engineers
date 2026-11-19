import cv2
import numpy as np
from picamera2 import Picamera2


WINDOW = "Surprise Rule Detection"
HSV_WINDOW = "HSV Controls"
RATIO_WINDOW = "Ratio Controls"
MIN_CONTOUR_AREA = 500


def create_hsv_trackbars():
    cv2.namedWindow(HSV_WINDOW)
    defaults = {
        "Green": (44, 75, 84, 255, 41, 255),
        "Red": (112, 128, 90, 255, 93, 255),
    }
    for colour, values in defaults.items():
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
                f"{colour} {name}", HSV_WINDOW, value, maximum, lambda _: None
            )


def read_hsv_bounds(colour):
    low = tuple(
        cv2.getTrackbarPos(f"{colour} low {channel}", HSV_WINDOW)
        for channel in "HSV"
    )
    high = tuple(
        cv2.getTrackbarPos(f"{colour} high {channel}", HSV_WINDOW)
        for channel in "HSV"
    )
    low = tuple(min(lower, upper) for lower, upper in zip(low, high))
    return np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)


def create_ratio_trackbars():
    cv2.namedWindow(RATIO_WINDOW)
    cv2.createTrackbar("Original min x100", RATIO_WINDOW, 35, 500, lambda _: None)
    cv2.createTrackbar("Original max x100", RATIO_WINDOW, 85, 500, lambda _: None)
    cv2.createTrackbar(
        "Minimum contour area",
        RATIO_WINDOW,
        MIN_CONTOUR_AREA,
        50000,
        lambda _: None,
    )


def read_ratio_controls():
    original_min = cv2.getTrackbarPos("Original min x100", RATIO_WINDOW) / 100.0
    original_max = cv2.getTrackbarPos("Original max x100", RATIO_WINDOW) / 100.0
    original_min = max(0.01, original_min)
    original_max = max(original_min, original_max)
    min_area = max(1, cv2.getTrackbarPos("Minimum contour area", RATIO_WINDOW))
    return original_min, original_max, min_area


def prepare_frame(frame):
    if frame is None:
        return None
    frame = frame[105:-1, 0:-1]
    if len(frame.shape) == 3 and frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return cv2.GaussianBlur(frame, (5, 5), 0)


def make_colour_mask(hsv_frame, colour):
    low, high = read_hsv_bounds(colour)
    mask = cv2.inRange(hsv_frame, low, high)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def find_largest_contour(mask, min_area):
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    candidates = [
        contour for contour in contours if cv2.contourArea(contour) >= min_area
    ]
    return max(candidates, key=cv2.contourArea, default=None)


def draw_detection(frame, detections, original_min, original_max):
    display = frame.copy()
    if not detections:
        cv2.putText(
            display,
            "No suitable block detected",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        return display

    colour, contour = max(detections, key=lambda item: cv2.contourArea(item[1]))
    x, y, width, height = cv2.boundingRect(contour)
    ratio = width / height if height else 0.0
    classification = (
        "original" if original_min <= ratio <= original_max else "modified"
    )
    draw_colour = (0, 255, 0) if colour == "Green" else (0, 0, 255)

    cv2.drawContours(display, [contour], -1, draw_colour, 2)
    cv2.rectangle(display, (x, y), (x + width, y + height), draw_colour, 2)
    cv2.putText(
        display,
        f"{colour} | {classification}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        draw_colour,
        2,
    )
    cv2.putText(
        display,
        f"width={width}px height={height}px ratio={ratio:.3f}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        draw_colour,
        2,
    )
    cv2.putText(
        display,
        f"original range={original_min:.2f}-{original_max:.2f}",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )
    print(
        f"{colour}: width={width}px height={height}px "
        f"ratio={ratio:.3f} -> {classification}"
    )
    return display


def main():
    create_hsv_trackbars()
    create_ratio_trackbars()

    camera = Picamera2()
    camera.start()
    try:
        while True:
            frame = prepare_frame(camera.capture_array("main"))
            if frame is None:
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_mask = make_colour_mask(hsv_frame, "Green")
            red_mask = make_colour_mask(hsv_frame, "Red")
            original_min, original_max, min_area = read_ratio_controls()

            detections = []
            green_contour = find_largest_contour(green_mask, min_area)
            red_contour = find_largest_contour(red_mask, min_area)
            if green_contour is not None:
                detections.append(("Green", green_contour))
            if red_contour is not None:
                detections.append(("Red", red_contour))

            display = draw_detection(
                frame, detections, original_min, original_max
            )
            cv2.imshow(WINDOW, display)
            cv2.imshow("Green mask", green_mask)
            cv2.imshow("Red mask", red_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
