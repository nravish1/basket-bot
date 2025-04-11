#runs on PC
import socket

HOST = '192.168.191.105'  # Raspberry Pi IP address
PORT = 8888

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print("Connected to Raspberry Pi")

# Make a file-like object to read one line at a time
sock_file = s.makefile('r')

while True:
    line = sock_file.readline()
    if not line:
        break  # connection closed

    try:
        distance = int(line.strip())  # Removes '\n' and converts to int
        print(f"Distance from LiDAR: {distance} cm")
    except ValueError:
        print("Invalid data:", repr(line))
