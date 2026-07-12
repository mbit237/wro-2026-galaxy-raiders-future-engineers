import rpicam

cam = rpicam.Rpicam()

while True:
    print(cam.detect_blob())