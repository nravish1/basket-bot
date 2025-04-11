import serial
import time

# file runs on raspberry pi
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

def read_lidar():
    while True:
        if ser.in_waiting >= 9:
            if ser.read() == b'\x59':
                if ser.read() == b'\x59':
                    data = ser.read(7)
                    distance = data[0] + data[1] * 256
                    strength = data[2] + data[3] * 256
                    print(f"Distance: {distance} cm, Strength: {strength}")
        else:
            time.sleep(0.05)

read_lidar()