# file runs on raspberry pi - CURRENT
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

#HOST = '192.168.191.126'  # Listen on all interfaces
HOST = '0.0.0.0'
PORT = 8888
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
print("Waiting for connection...")
conn, addr = s.accept()
print(f"Connected by {addr}")


def read_lidar():
    while True:
        if ser.in_waiting >= 9:
            if ser.read() == b'\x59':
                if ser.read() == b'\x59':
                    data = ser.read(7)
                    low = data[0]
                    high = data[1]
                    dist = (high << 8) | low
                    return dist
                    # print(f"Distance: {distance} cm, Strength: {strength}")
        else:
            time.sleep(0.05)

while True:
    distance = read_lidar()
    message = f"{distance}\n"  # Add newline to clearly separate each message
    conn.sendall(message.encode('utf-8'))

# read_lidar()
