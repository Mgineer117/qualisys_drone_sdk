"""
Updated Crazyflie circular trajectory using remote target computation API
"""

from time import time

import pynput

from api.schema import TargetRequest
from api.service import get_target_position
from qfly import Pose, QualisysCrazyflie, World

# SETTINGS
cf_body_name = "flapper"
cf_uri = "radio://0/80/2M/E7E7E7E7E7"
cf_marker_ids = [1, 2, 3, 4]
circle_radius = 0.75
circle_axis = "XYZ"
circle_speed_factor = 9
qtm_ip = "128.174.245.190"

sampling_rate = 0.1
last_key_pressed = None


def on_press(key):
    global last_key_pressed
    last_key_pressed = key


listener = pynput.keyboard.Listener(on_press=on_press)
listener.start()

world = World(expanse=1.8, speed_limit=1.1)
# preflight check
req = TargetRequest(
    dt=0,
    axis=circle_axis,
    radius=circle_radius,
    speed=circle_speed_factor,
    origin_x=world.origin.x,
    origin_y=world.origin.y,
    origin_z=world.origin.z,
    x_cur=world.origin.x,
    y_cur=world.origin.y,
    z_cur=world.origin.z,
    rot_mat=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
)
target, status = get_target_position(req)
if status != "OK":
    print(f"Error: {status}")
    exit(1)

with QualisysCrazyflie(
    cf_body_name, cf_uri, world, marker_ids=cf_marker_ids, qtm_ip=qtm_ip
) as qcf:
    t0 = time()
    data = {"pose": [], "time": [], "control": []}
    # (logging setup omitted for brevity)

    while qcf.is_safe():
        if last_key_pressed == pynput.keyboard.Key.esc:
            break
        dt = time() - t0

        if dt < 5:
            target = Pose(world.origin.x, world.origin.y, 1.0)
        elif dt < 20:
            # fetch from remote service
            req = TargetRequest(
                dt=dt,
                axis=circle_axis,
                radius=circle_radius,
                speed=circle_speed_factor,
                origin_x=world.origin.x,
                origin_y=world.origin.y,
                origin_z=world.origin.z,
                x_cur=qcf.pose.x,
                y_cur=qcf.pose.y,
                z_cur=qcf.pose.z,
                rot_mat=qcf.pose.rot_mat.tolist(),
            )
            target, status = get_target_position(req)
            if status != "OK":
                print(f"Error: {status}")
                break
        qcf.safe_position_setpoint(target)
    # Land
    first_z = qcf.pose.z
    landing_time = 5
    start_time = time()
    while qcf.is_safe():
        if dt - start_time < 5:
            print(f"[t={dt}] Maneuvering - Center...")
            # Set target
            target = Pose(world.origin.x, world.origin.y, 1.0)
            # Engage
            qcf.safe_position_setpoint(target)
        elif qcf.pose.z > 0.40:
            print(qcf.pose.z)
            print("landing...")
            cur_time = time()
            target = Pose(
                world.origin.x,
                world.origin.y,
                max(-0.20, first_z * (1 - (cur_time - start_time) / landing_time)),
            )
            qcf.safe_position_setpoint(target)
        else:
            print("landed")
            break
