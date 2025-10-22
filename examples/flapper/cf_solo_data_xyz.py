"""
qfly | Qualisys Drone SDK Example Script: Solo Crazyflie

Takes off, flies circles around Z, Y, X axes.
ESC to land at any time.
"""

import os
import json
import pynput
from datetime import datetime
from time import sleep, time
from functools import partial
from math import floor
import matplotlib.pyplot as plt

from cflib.crazyflie.syncLogger import SyncLogger


from qfly import Pose, QualisysCrazyflie, World, utils
from functions import generate_reference_trajectory, get_logger

# SETTINGS
cf_body_name = "flapper"  # QTM rigid body name
cf_uri = "radio://0/80/2M/E7E7E7E701"  # Crazyflie address
cf_marker_ids = [1, 2, 3, 4]  # Active marker IDs
qtm_ip = "128.174.245.190"

MAX_TIME = 30  # s
TIME_INTERVAL = 0.1  # s
HORIZON = int(MAX_TIME // TIME_INTERVAL)
TIME_MARGIN = 1  # s

# circle_radius = 0.5  # Radius of the circular flight path
# circle_speed_factor = 0.12  # How fast the Crazyflie should move along circle


# Watch key presses with a global variable
last_key_pressed = None


# Set up keyboard callback
def on_press(key):
    """React to keyboard."""
    global last_key_pressed
    last_key_pressed = key
    if key == pynput.keyboard.Key.esc:
        fly = False


def log_callback(timestamp, data, logconf, data_log, key):
    print(f"{timestamp}, {data}, {logconf.name}")
    data_log[key].append(data)


# Listen to the keyboard
listener = pynput.keyboard.Listener(on_press=on_press)
listener.start()


# Set up world - the World object comes with sane defaults
world = World()


# Set up the asynchronous log configuration
# For details, see https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/

group_list = [
    "stabilizer",
    "pos",
    "vel",
    "acc",
    "attitude_rate",
    "motor",
    "motor_req",
    "gyro",
    "target_pos",
    "controller_cmd",
    "controller_attitude",
    "controller_attitude_rate",
]
conf_list = get_logger(group_list, TIME_INTERVAL)

# Prepare for liftoff
with QualisysCrazyflie(
    cf_body_name, cf_uri, world, marker_ids=cf_marker_ids, qtm_ip=qtm_ip
) as qcf:

    # === Get current xyz === #
    init_x = qcf.pose.x
    init_y = qcf.pose.y
    init_z = qcf.pose.z

    # === Generate reference trajectory with sinusoidal function === #
    traj_x, traj_y, traj_z, seed = generate_reference_trajectory(
        init_x, init_y, init_z, HORIZON, TIME_INTERVAL, visualize=True
    )

    # === Initialize data logging === #
    data = {}
    # data["radius"] = circle_radius
    # data["angular_speed"] = circle_speed_factor
    # data["save_freq"] = save_freq
    # data["pid_gains"] = pid_gains
    data["pose"] = []
    data["time"] = []
    data["target"] = []

    # Asynchronous logging from the flapper firmware
    for group in group_list:
        data[group] = []
    data["time"] = []
    for logconf in conf_list:
        qcf.cf.log.add_config(logconf)
    for group, logconf in zip(group_list, conf_list):
        callback = partial(log_callback, data_log=data, key=group)
        logconf.data_received_cb.add_callback(callback)
        logconf.start()

    # === Begin maneuvers === #
    print("Beginning maneuvers...")
    t, last_index = time(), -1
    while qcf.is_safe() and dt <= (MAX_TIME + TIME_MARGIN):
        # Mind the clock
        dt = time() - t
        time_index = int(max(0, min(floor(dt / TIME_INTERVAL), HORIZON - 1)))
        sim_dt = time_index * TIME_INTERVAL

        # Set target based on time index by ensuring time index is within bounds
        target = Pose(
            traj_x[time_index],
            traj_y[time_index],
            traj_z[time_index],
        )

        # Execute control
        qcf.safe_position_setpoint(target)

        # log information just once for each time index
        if dt > sim_dt and last_index != time_index:
            last_index = time_index
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
            data["time"].append(sim_dt)
            data["target"].append((target.x, target.y, target.z))
            last_saved_t = time()

        # Print status information
        print(f"[INFO]: Time elapsed: {dt} s, target index: {time_index}")
        if dt >= MAX_TIME:
            print(f"[INFO]: Time limit reached, preparing to land in {TIME_MARGIN} s.")
            break

        # Terminate upon Esc command
        if last_key_pressed == pynput.keyboard.Key.esc:
            print(last_key_pressed)
            break

    # === Stop logging data from the flapper firmware === #
    for logconf in conf_list:
        logconf.stop()

    # === Initiate Landing === #
    while qcf.pose.z > 0.5:
        print(qcf.pose.z)
        print("landing...")
        qcf.land_in_place()

    # Save the logged data to a JSON file
    current_time = time()
    dt_now = datetime.fromtimestamp(current_time)
    dir_path = f"data/flapper/{dt_now.year}_{dt_now.month}_{dt_now.day}"
    os.makedirs(dir_path, exist_ok=True)

    formatted_time = dt_now.strftime("%Y%m%d%H%M%S")
    with open(f"{dir_path}/{seed}_{formatted_time}.json", "w") as file:
        json.dump(data, file, indent=4)
