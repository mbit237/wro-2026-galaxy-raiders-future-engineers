import pigpio

class Encoder:
    def __init__(self, pi):
        self.cb1 = pi.callback(5, pigpio.RISING_EDGE, self.step_count)
        self.cb2 = pi.callback(6, pigpio.EITHER_EDGE, self.drive_dir)
        self.steps = 0
        self.pin6_level = False
        
    # gpio -- pin 
    # level -- rising / falling edge 
    # tick -- The number of microseconds since boot
    def step_count(self, gpio, level, tick):
        if self.pin6_level:
            self.steps += 1
        else:
            self.steps -= 1

    def drive_dir(self, gpio, level, tick):
        if level == 1:
            self.pin6_level = True 
        elif level == 0:
            self.pin6_level = False 