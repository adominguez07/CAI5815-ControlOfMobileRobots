import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 230400   # LD06 / LD19 default baud rate

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Listening for raw data...\n")

try:
    while True:
        data = ser.read(100)
        if data:
            print(data)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")
    ser.close()