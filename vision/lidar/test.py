import socket

HOST = '192.168.191.105'  # Pi’s IP
PORT = 8888

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print("Connected to Pi!")

sock_file = s.makefile()  # Treat socket as a file-like object

while True:
    line = sock_file.readline()
    if line:
        distance = int(line.strip())
        print(f"Distance from LiDAR: {distance} cm")
