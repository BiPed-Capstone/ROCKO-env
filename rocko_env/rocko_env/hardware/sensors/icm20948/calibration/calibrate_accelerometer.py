'''
Custom script meant to be run once to generate accelerometer hard offset data.
Should be carried out with all hardware installed on the robot sans the legs
for convenience. 

Adapted from the following jupyter notebook: https://github.com/adafruit/Adafruit_SensorLab/blob/master/notebooks/acc_Gyro_Calibration.ipynb 
'''
import board
import numpy as np
import adafruit_icm20x
from time import sleep
from tqdm import tqdm
from ahrs.filters import Madgwick
from adafruit_icm20x import AccelRange, GyroRange

class AccCalibrator:
    def __init__(self, memory=1000):
            i2c = board.I2C()   # uses board.SCL and board.SDA
            self.icm = adafruit_icm20x.ICM20948(i2c)
            self.icm.gyro_data_rate_divisor = 0
            self.icm.accelerometer_data_rate_divisor = 0
            self.icm.accelerometer_range = AccelRange.RANGE_4G
            self.icm.gyro_range = GyroRange.RANGE_500_DPS
            
            self.memory = memory
            self.gyro_arr = np.zeros((self.memory, 3))
            self.acc_arr = np.zeros((self.memory, 3))

    def capture_dataset(self):
        print('Starting accelerometer capture.')   
        for i in range(0, self.memory):
            # Add data to sample arrays
            self.acc_arr[i] = self.icm.acceleration
            # Wait for 10ms for new data to be gathered
            sleep(0.01) 
        print('Accelerometer capture done.')   

    def calibrate_dataset(self):
        min_x = min(t[0] for t in self.acc_arr)
        min_y = min(t[1] for t in self.acc_arr)
        min_z = min(t[2] for t in self.acc_arr)

        max_x = max(t[0] for t in self.acc_arr)
        max_y = max(t[1] for t in self.acc_arr)
        max_z = max(t[2] for t in self.acc_arr)

        acc_calibration = [ (max_x + min_x) / 2, (max_y + min_y) / 2, (max_z + min_z) / 2]

        # write acc_calibration to file
        with open('hard_offset', 'w') as f:
            for i in range(3):
                f.write(f"{acc_calibration[i]}\n")
                
    def capture_gyro_accel_data(self):
        # measure 1000 samples of gyro and accelerometer
        madgwick = Madgwick()
        with open('gyro_acc_data.txt', 'w') as f:
            prev_q = [0.7071, 0.0, 0.7071, 0.0]
            current_q = []
            for i in tqdm(range(self.memory)):
                current_q = madgwick.updateIMU(q=prev_q, gyr=self.icm.gyro, acc=self.icm.acceleration)  
                f.write(f"{self.icm.gyro},{self.icm.acceleration}\n")
                prev_q = current_q
                sleep(0.02)
        
        print("Below is your calibrated 0. Please paste it in ICM20948.py")
        print(str(current_q))
                
