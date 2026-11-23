import serial
import threading
import time

PORTS = {
    "UPPER_ARM": "/dev/ttyACM0",
    "FOREARM": "/dev/ttyACM1",
}

def read_sensor(name, port):
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print(f"{name} connected to {port}")

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
                    w, x, y, z = map(float, parts[1:])
                except ValueError:
                    continue

                print(
                    f"{name}: "
                    f"w={w:.4f} x={x:.4f} "
                    f"y={y:.4f} z={z:.4f}"
                )

    except serial.SerialException as e:
        print(f"{name} ERROR: {e}")

threads = []

for name, port in PORTS.items():
    thread = threading.Thread(
        target=read_sensor,
        args=(name, port),
        daemon=True
    )
    thread.start()
    threads.append(thread)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopped.")
