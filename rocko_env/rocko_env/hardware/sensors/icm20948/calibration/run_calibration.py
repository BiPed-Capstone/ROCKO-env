from calibrate_accelerometer import AccCalibrator

input('Press any key to begin calibration.')
calibrator = AccCalibrator
calibrator.capture_dataset()
calibrator.calibrate_dataset()