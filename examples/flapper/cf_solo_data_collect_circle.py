"""
qfly | Qualisys Drone SDK Example Script: Solo Crazyflie

Takes off, flies circles around Z, Y, X axes.
ESC to land at any time.
"""

import json
from datetime import datetime
from time import sleep, time
import numpy as np
import pynput
from qfly import Pose, QualisysCrazyflie, World, utils

# SETTINGS
cf_body_name = "flapper"  # QTM rigid body name
cf_uri = "radio://0/80/2M/E7E7E7E701"  # Crazyflie address
cf_marker_ids = [1, 2, 3, 4]  # Active marker IDs
circle_radius = 0.5  # Radius of the circular flight path
circle_speed_factor = 15  # How fast the Crazyflie should move along circle [degree/s]
qtm_ip = "128.174.245.190"

# Watch key presses with a global variable
last_key_pressed = None


# Set up keyboard callback
def on_press(key):
    """React to keyboard."""
    global last_key_pressed
    last_key_pressed = key
    if key == pynput.keyboard.Key.esc:
        fly = False


# Listen to the keyboard
listener = pynput.keyboard.Listener(on_press=on_press)
listener.start()


# Set up world - the World object comes with sane defaults
world = World(expanse=5.0, speed_limit=1.1)

# Prepare for liftoff
with QualisysCrazyflie(
    cf_body_name, cf_uri, world, marker_ids=cf_marker_ids, qtm_ip=qtm_ip
) as qcf:

    # Let there be time
    t = time()
    last_saved_t = time()
    save_freq = 0.01
    dt = 0

    print("Beginning maneuvers...")
    data = {}
    data["radius"] = circle_radius
    data["angular_speed"] = circle_speed_factor
    data["save_freq"] = save_freq
    data["pose"] = []
    data["time"] = []
    data["control"] = []
    # MAIN LOOP WITH SAFETY CHECK
    while qcf.is_safe():

        # Terminate upon Esc command
        if last_key_pressed == pynput.keyboard.Key.esc:
            break
        print(last_key_pressed)
        # Mind the clock
        dt = time() - t

        # Calculate Crazyflie's angular position in circle, based on time
        phi = circle_speed_factor * dt

        # Take off and hover in the center of safe airspace for 5 seconds
        if dt < 5:
            print(f"[t={dt}] Maneuvering - Center...")
            # Set target
            target = Pose(world.origin.x, world.origin.y, 1.0)
            # Engage
            qcf.safe_position_setpoint(target)
        # Move out and circle around Z axis
        # elif dt < 40:
        #     print(f"[t={dt}] Maneuvering - Circle around Z...")
        #     # Set target
        #     _x, _y = utils.pol2cart(circle_radius, phi)
        #     target = Pose(world.origin.x + _x, world.origin.y + _y, 1.0)
        #     # Engage
        #     qcf.safe_position_setpoint(target)
        elif dt < 40:
            print(f'[t={int(dt)}] Maneuvering - Circle around X...')
            # Set target
            _y, _z = utils.pol2cart(circle_radius, 5 * phi)
            _x, _y = utils.pol2cart(circle_radius * 1.75, phi)
            target = Pose(world.origin.x + _x,
                          world.origin.y + _y,
                          1.25 + _z)
            # Engage
            qcf.safe_position_setpoint(target)
        # elif dt < 40:
        #     print(f'[t={int(dt)}] Maneuvering - Circle around Y...')
        #     # Set target
        #     _x, _z = utils.pol2cart(circle_radius, phi)
        #     target = Pose(world.origin.x + _x,
        #                   world.origin.y,
        #                   1.25 + _z)
        #     # Engage
        #     qcf.safe_position_setpoint(target)
        # Back to center
        elif dt < 47:
            print(f"[t={dt}] Maneuvering - Center...")
            # Set target
            target = Pose(world.origin.x, world.origin.y, 1.0)
            # Engage
            qcf.safe_position_setpoint(target)
                # Move out and circle around Y axis
        else:
            break
        
        if time() - last_saved_t > save_freq:
            data["pose"].append(
                (
                    qcf.pose.x,
                    qcf.pose.y,
                    qcf.pose.z,
                    qcf.pose.roll,
                    qcf.pose.pitch,
                    qcf.pose.yaw,
                    qcf.pose.rotmatrix,
                )
            )
            data["time"].append(time())
            data["control"].append((target.x, target.y, target.z))
            last_saved_t = time()
        if not qcf.is_safe():
            print("not safe")
        """


        # Back to center
        elif dt < 45:
            print(f'[t={int(dt)}] Maneuvering - Center...')
            # Set target
            target = Pose(world.origin.x, world.origin.y, 1.0)
            # Engage
            qcf.safe_position_setpoint(target)

        # Move and circle around X axis
        elif dt < 60:
            print(f'[t={int(dt)}] Maneuvering - Circle around X...')
            # Set target
            _y, _z = utils.pol2cart(circle_radius, phi)
            target = Pose(world.origin.x,
                          world.origin.y + _y,
                          1.0 + _z)
            # Engage
            qcf.safe_position_setpoint(target)
        
        # Back to center
        elif dt < 65:
            print(f'[t={int(dt)}] Maneuvering - Center...')
            # Set target
            target = Pose(world.origin.x, world.origin.y, 1.0)
            # Engage
            qcf.safe_position_setpoint(target)
        """

    # Land
    first_z= qcf.pose.z
    landing_time = 5
    start_time = time()
    while qcf.pose.z > 0.40:
        print(qcf.pose.z)
        print("landing...")
        cur_time = time()
        target = Pose(qcf.pose.x, qcf.pose.y, max(-0.20, first_z * (1 - (cur_time - start_time) / landing_time)))
        qcf.safe_position_setpoint(target)
# Get the current time in seconds since the epoch
current_time = time()

# Convert the time to a human-readable format
formatted_time = datetime.fromtimestamp(current_time).strftime("%Y%m%d%H%M%S")
with open(formatted_time + ".json", "w") as file:
    json.dump(data, file, indent=4)
