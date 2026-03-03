from adafruit_rplidar import RPLidar
import time

PORT = "/dev/ttyUSB0"

lidar = RPLidar(None, PORT)

try:
    print("Resetting lidar...")
    lidar.reset()
    time.sleep(2)

    print("Starting motor...")
    lidar.start_motor()
    time.sleep(2)

    print("Trying scan...")
    for scan in lidar.iter_scans():
        print("Scan received:", len(scan))
        print(scan[:5])
        break

except Exception as e:
    print("Error:", e)

finally:
    print("Stopping...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()