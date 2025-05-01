import board
import adafruit_bno055

def main():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    
    while True:
        print("Pitch: {}".format(sensor.euler[2]))
        
main()