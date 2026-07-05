import gyro 
import coind4
import pigpio
import encoder
import drive
import camera_servo
import led

def init():
    gyro_device = gyro.Gyro()
    gyro_device.load_calibration()

    lidar = coind4.CoinD4()
    lidar.start()

    pi = pigpio.pi()
    encoder_device = encoder.Encoder(pi)
    drive_device = drive.Drive(pi)
    camera_servo_device = camera_servo.CameraServo(pi)
    led_device = led.LED(pi)
    
    return {
        "gyro": gyro_device,
        "lidar": lidar, 
        "encoder": encoder_device, 
        "drive": drive_device,
        "pi": pi,
        "camera_servo": camera_servo_device,
        "led": led_device
    }