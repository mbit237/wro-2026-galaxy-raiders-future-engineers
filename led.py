import pigpio

class LED:
    def __init__(self, pi):
        self.pi = pi
        self.pi.set_mode(13, pigpio.OUTPUT) # red
        self.pi.set_mode(19, pigpio.OUTPUT) # green
        self.pi.set_mode(26, pigpio.OUTPUT) # yellow

    def red_on(self):
        self.pi.write(13, 1)

    def red_off(self):
        self.pi.write(13, 0)

    def green_on(self):
        self.pi.write(19, 1)

    def green_off(self):
        self.pi.write(19, 0)

    def yellow_on(self):
        self.pi.write(26, 1)

    def yellow_off(self):
        self.pi.write(26, 0)
        
    def all_off(self):
        self.red_off()
        self.green_off()
        self.yellow_off()