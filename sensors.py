def read(devices):
    if devices["lidar"].update():
        lidar_measurements = devices["lidar"].get_measurements()
    else:
        lidar_measurements = False
        
    return {
        "gyro": devices["gyro"].delta_z(),
        "lidar": lidar_measurements,
        "encoder": devices["encoder"].steps()
    }
