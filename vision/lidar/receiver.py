# PC: receiver.py
import socket

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5001       # Must match port used by Pi

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Waiting for connection on port {PORT}...")
conn, addr = server_socket.accept()
print(f"Connected by {addr}")

try:
    while True:
        data = conn.recv(1024).decode()
        if not data:
            break
        print(f"Received distance: {data.strip()} cm")  # Or inches if you're using that

finally:
    conn.close()
    server_socket.close()
    