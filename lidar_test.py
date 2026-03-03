from adafruit_rplidar import RPLidar
import time

PORT = "/dev/ttyUSB0"

for b in [256000, 115200, 230400]:
    print("\nTrying baud:", b)
    lidar = RPLidar(None, PORT, baudrate=b)
    try:
        lidar.start_motor()
        time.sleep(2)
        for scan in lidar.iter_scans():
            print("SUCCESS baud", b, "| points:", len(scan), "| sample:", scan[:5])
            raise SystemExit
    except Exception as e:
        print("Fail baud", b, "->", e)
    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass