import socket

HOST = '192.168.191.105'  # Raspberry Pi IP address
PORT = 8888

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

print("Test")

while True:
    data = s.recv(1024).decode('utf-8')  # Receive the data (bytes, no decoding)
    
    if data:
        distance = int(data.decode('utf-8'))  # Convert the raw bytes to an integer (distance)
        print(f"Distance from LiDAR: {distance} cm")
