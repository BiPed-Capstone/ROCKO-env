'''
Custom script meant to be run once to generate magnetometer hard offset data.
Should be carried out with all hardware installed on the robot sans the legs
for convenience. 

Outputs "hard_offset" within the same directory.

Adapted from the following jupyter notebook: https://github.com/adafruit/Adafruit_SensorLab/blob/master/notebooks/Mag_Gyro_Calibration.ipynb 
'''
import board
import numpy as np
import adafruit_icm20x
from time import sleep

def __init__(self, memory=None):
        i2c = board.I2C()   # uses board.SCL and board.SDA
        self.icm = adafruit_icm20x.ICM20948(i2c)
        if memory == None:
            self.memory = 2500
        else:
            self.memory = memory
        self.mag_arr = np.zeros((self.memory, 3))

def capture_dataset(self):
    self.get_logger().info('Starting magnetometer capture.')   
    for i in range(0, self.memory):
        # Add data to sample arrays
        self.mag_arr[i] = self.icm.magnetic
        # Wait for 10ms for new data to be gathered
        sleep(0.01) 
    self.get_logger().info('Magnetometer capture done.')   

def calibrate_dataset(self):
    min_x = min(t[0] for t in self.mag_arr)
    min_y = min(t[1] for t in self.mag_arr)
    min_z = min(t[2] for t in self.mag_arr)

    max_x = max(t[0] for t in self.mag_arr)
    max_y = max(t[1] for t in self.mag_arr)
    max_z = max(t[2] for t in self.mag_arr)

    mag_calibration = [ (max_x + min_x) / 2, (max_y + min_y) / 2, (max_z + min_z) / 2]

    # write mag_calibration to file
    with open('hard_offset', 'w') as f:
          for i in range(3):
            f.write(f"{mag_calibration[i]}\n")