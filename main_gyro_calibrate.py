import gyro 

gyro_device = gyro.Gyro()
print(f"z_error: {gyro_device.calibration()}")
gyro_device.save_calibration()
print("Save calibration")
