STEER_MAX = 45
CENTER = 45

#GPIO20, GPIO21

class Drive:
    def __init__(self, pi):
        self.pi = pi
        pi.set_PWM_frequency(20, 200)

    def drive(self, speed): # 0-255
        if speed > 0:
            self.pi.set_PWM_dutycycle(20, 255-speed)
            self.pi.set_PWM_dutycycle(21, 255)
        else:
            self.pi.set_PWM_dutycycle(20, 255)
            self.pi.set_PWM_dutycycle(21, 255+speed)

    def steering(self, dir):
        if dir < -STEER_MAX:
            dir = -STEER_MAX
        elif dir > STEER_MAX:
            dir = STEER_MAX
        pulse_duration = 1340 + (80 / 9) *(dir + CENTER) - 400
        self.pi.set_servo_pulsewidth(23, pulse_duration)

    def steer_p(self, dir, curr_angle, speed):
        while curr_angle - dir > 180:
            curr_angle -= 360
        while curr_angle - dir < -180:
            curr_angle += 360

        error = curr_angle - dir
        gain = 2 
        correction = error * gain 
        self.steering(correction)
        self.drive(speed)

    def steer_p_back(self, dir, curr_angle, speed):
        while curr_angle - dir > 180:
            curr_angle -= 360
        while curr_angle - dir < -180:
            curr_angle += 360

        error = curr_angle - dir
        gain = -2 
        correction = error * gain 
        self.steering(correction)
        self.drive(-speed) 
