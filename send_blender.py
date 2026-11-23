import serial
import socket
import threading
import json
import time

PORTS = {
    "Bone": "/dev/ttyACM0",
    "Bone.001": "/dev/ttyACM1",
}

UDP_HOST = "172.31.224.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def read_sensor(bone_name, port):
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print(f"{bone_name} connected to {port}")

            while True:
                line = ser.readline().decode(
                    "utf-8", errors="ignore"
                ).strip()

                if not line.startswith("Q,"):
                    continue

                parts = line.split(",")

                if len(parts) != 5:
                    continue

                try:
                    q = [float(v) for v in parts[1:]]
                except ValueError:
                    continue

                packet = {
                    "bone": bone_name,
                    "q": q
                }

                sock.sendto(
                    json.dumps(packet).encode("utf-8"),
                    (UDP_HOST, UDP_PORT)
                )

    except serial.SerialException as e:
        print(f"{bone_name}: {e}")

for bone_name, port in PORTS.items():
    threading.Thread(
        target=read_sensor,
        args=(bone_name, port),
        daemon=True
    ).start()

print("Sending sensor data to Blender on UDP port 5005")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped")
