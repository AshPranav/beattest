
import bpy
import socket
import json
from mathutils import Quaternion

UDP_PORT = 5005

# Stop previous receiver
old = bpy.app.driver_namespace.get("imu_receiver")
if old:
    old["running"] = False
    try:
        old["socket"].close()
    except OSError:
        pass

arm = next(
    obj for obj in bpy.data.objects
    if obj.type == "ARMATURE"
)

upper = arm.pose.bones["Bone"]
forearm = arm.pose.bones["Bone.001"]

upper.rotation_mode = "QUATERNION"
forearm.rotation_mode = "QUATERNION"

# Preserve initial bone pose
upper_initial = upper.rotation_quaternion.copy()
forearm_initial = forearm.rotation_quaternion.copy()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(False)

state = {
    "running": True,
    "socket": sock,
    "latest": {},
    "reference": {},
}

bpy.app.driver_namespace["imu_receiver"] = state


def update_imu():
    if not state["running"]:
        return None

    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except BlockingIOError:
            break
        except OSError:
            return None

        try:
            packet = json.loads(data.decode())
            name = packet["bone"]
            values = packet["q"]

            if name in ("Bone", "Bone.001") and len(values) == 4:
                q = Quaternion(values)
                q.normalize()
                state["latest"][name] = q

        except (ValueError, KeyError, TypeError):
            continue

    if not all(
        name in state["latest"]
        for name in ("Bone", "Bone.001")
    ):
        return 0.02

    q_upper = state["latest"]["Bone"]
    q_forearm = state["latest"]["Bone.001"]

    # Initial calibration
    if not state["reference"]:
        state["reference"]["Bone"] = q_upper.copy()
        state["reference"]["Bone.001"] = q_forearm.copy()
        print("Both sensors calibrated")

    ref_upper = state["reference"]["Bone"]
    ref_forearm = state["reference"]["Bone.001"]

    # Upper-arm motion from calibrated orientation
    upper_relative = ref_upper.inverted() @ q_upper

    # Forearm orientation relative to upper arm
    elbow_current = q_upper.inverted() @ q_forearm
    elbow_reference = ref_upper.inverted() @ ref_forearm
    elbow_relative = elbow_reference.inverted() @ elbow_current

    upper.rotation_quaternion = (
        upper_initial @ upper_relative
    )

    forearm.rotation_quaternion = (
        forearm_initial @ elbow_relative
    )

    return 0.02


bpy.app.timers.register(update_imu, first_interval=0.1)

print("Parent-relative IMU tracking started")
