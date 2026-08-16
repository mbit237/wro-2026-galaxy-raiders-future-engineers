import src.rpicam as rpicam

cam = rpicam.Rpicam()

while True:
    print(cam.detect_blob())