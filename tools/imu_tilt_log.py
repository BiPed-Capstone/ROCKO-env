"""Log the BNO055's Euler angles and all three gyro axes for N seconds (gyro check, proposal 0008 A).
On the Pi, as your user, from the repo root (needs ~/.venv with gpiod and adafruit-circuitpython-bno055):
    cd ~/ROCKO-env && ~/.venv/bin/python3 tools/imu_tilt_log.py 40 /tmp/tilt.csv
The IMU must be free (tank mode, or the robot service stopped). The CSV keeps the first rows, which read all
zeros while the sensor starts: drop rows where roll_deg and pitch_deg are both 0 before analysing.
Columns: t_s, yaw_deg, roll_deg, pitch_deg (raw sensor, without the xacro's pitch_offset_deg), gx/gy/gz_deg_s."""
import math, sys, time
import board, adafruit_bno055

seconds, out = float(sys.argv[1]), sys.argv[2]
imu = adafruit_bno055.BNO055_I2C(board.I2C())
t0 = time.monotonic()
with open(out, "w") as f:
    f.write("t_s,yaw_deg,roll_deg,pitch_deg,gx_deg_s,gy_deg_s,gz_deg_s\n")
    while time.monotonic() - t0 < seconds:
        g = imu.gyro            # (x, y, z) rad/s
        e = imu.euler           # (yaw, roll, pitch) deg
        t = time.monotonic() - t0
        if g is None or e is None or None in g or None in e:
            continue
        f.write(f"{t:.3f},{e[0]:.3f},{e[1]:.3f},{e[2]:.3f},"
                + ",".join(f"{math.degrees(v):.3f}" for v in g) + "\n")
        time.sleep(0.04)
print("done", out)
