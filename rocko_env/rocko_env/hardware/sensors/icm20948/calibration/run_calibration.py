from calibrate_magnetometer import MagCalibrator

input('Press any key to begin calibration.')
MagCalibrator.capture_dataset()
MagCalibrator.calibrate_dataset()