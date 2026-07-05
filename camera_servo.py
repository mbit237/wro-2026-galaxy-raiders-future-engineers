STEER_MAX = 89
CENTER_US = 1475 #center microseconds

#GPIO20, GPIO21

class CameraServo:
    def __init__(self, pi):
        self.pi = pi

    def set_dir(self, dir):
        if dir < -STEER_MAX:
            dir = -STEER_MAX
        elif dir > STEER_MAX:
            dir = STEER_MAX
        pulse_duration = CENTER_US + (1000 / 90) * dir
        self.pi.set_servo_pulsewidth(24, pulse_duration)
