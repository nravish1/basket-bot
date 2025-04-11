# lidar_sender.py on Raspberry Pi
import socket
import time

def get_lidar_data():
    # Replace this with your actual LiDAR reading logic
    return "Sample Lidar Data"

HOST = '192.168.185.126'  # Replace with your PC's IP
PORT = 5005

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("Connected to PC")
    while True:
        data = get_lidar_data()
        s.sendall(data.encode())
        time.sleep(0.05)  # adjust as needed
