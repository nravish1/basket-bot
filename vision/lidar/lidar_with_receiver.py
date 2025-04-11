import serial
import time
import socket


# file runs on raspberry pi***
ser = serial.Serial('/dev/serial0', 115200, timeout=1)


HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5001       # Must match port used by Pi

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print("Waiting for connection...")
conn, addr = s.accept()
print(f"Connected by {addr}")


def read_lidar():
    while True:
        if ser.in_waiting >= 9:
            if ser.read() == b'\x59':
                if ser.read() == b'\x59':
                    data = ser.read(7)
                    dist = struct.unpack('<H', data[0:2])[0]  # Distance in cm
                    return dist

                    # print(f"Distance: {distance} cm, Strength: {strength}")
        else:
            time.sleep(0.05)

while True:
    distance = read_lidar()
    message = str(distance)
    conn.sendall(message.encode('utf-8'))

# read_lidar()