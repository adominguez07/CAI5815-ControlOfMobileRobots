from adafruit_rplidar import RPLidar
import time

PORT = "/dev/ttyUSB0"

lidar = RPLidar(None, PORT, baudrate=256000)

try:
    print("Starting motor...")
    lidar.start_motor()
    time.sleep(2)

    print("Reading scans...")
    for scan in lidar.iter_scans():
        print("Scan received:", len(scan))
        print(scan[:5])  # print first 5 measurements
        break

except Exception as e:
    print("Error:", e)

finally:
    print("Stopping...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()