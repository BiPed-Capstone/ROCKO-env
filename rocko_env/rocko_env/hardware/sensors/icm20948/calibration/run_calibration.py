import calibrate_magnetometer

input('Press any key to begin calibration.')
calibrate_magnetometer.capture_dataset()
calibrate_magnetometer.calibrate_dataset()