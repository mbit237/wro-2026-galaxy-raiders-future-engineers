import initialise_hardware
devices = initialise_hardware.init()
devices["drive"].drive(0)
devices["drive"].steering(0)
