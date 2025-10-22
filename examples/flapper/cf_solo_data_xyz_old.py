"""
qfly | Qualisys Drone SDK Example Script: Solo Crazyflie

Takes off, flies circles around Z, Y, X axes.
ESC to land at any time.
"""

import json
import random
import numpy as np
from time import sleep, time
import matplotlib.pyplot as plt

import pynput

from qfly import Pose, QualisysCrazyflie, World, utils

# SETTINGS
cf_body_name = "flapper"  # QTM rigid body name
cf_uri = "radio://0/80/2M/E7E7E7E701"  # Crazyflie address
cf_marker_ids = [1, 2, 3, 4]  # Active marker IDs
qtm_ip = "128.174.245.190"

TIME_MARGIN = 5  # s
MAX_TIME = 30  # s
TIME_INTERVAL = 0.1  # s
HORIZON = int(MAX_TIME // TIME_INTERVAL)

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


# Listen to the keyboard
listener = pynput.keyboard.Listener(on_press=on_press)
listener.start()


# Set up world - the World object comes with sane defaults
world = World()

# Prepare for liftoff
with QualisysCrazyflie(
    cf_body_name, cf_uri, world, marker_ids=cf_marker_ids, qtm_ip=qtm_ip
) as qcf:

    def get_state(qcf: QualisysCrazyflie, time_taken: float) -> np.ndarray:
        """Determine current state of the Crazyflie."""
        pose = qcf.pose
        velocity = qcf.velocity
        attitude = qcf.attitude
        angular_velocity = qcf.angular_velocity

        state = np.array(
            [
                pose.x,
                pose.y,
                pose.z,
                velocity.x,
                velocity.y,
                velocity.z,
                attitude.roll,
                attitude.pitch,
                attitude.yaw,
            ]
        )
        return state

    # Let there be time
    t = time()
    dt = 0

    # get current xyz
    init_x = qcf.pose.x
    init_y = qcf.pose.y
    init_z = qcf.pose.z

    # generate random x, y, delta z for specifying target
    delta_z = random.uniform(0.5, 1.0)

    target_x = init_x + random.uniform(-0.5, 0.5)
    target_y = init_y + random.uniform(-0.5, 0.5)
    target_z = init_z + delta_z

    # generate reference trajectory with sinusoidal function
    amplitude = 0.1
    random_frequency = random.uniform(0.1, 0.5)
    traj_x, traj_y, traj_z = [], [], []
    for i in range(HORIZON):
        traj_x.append(
            init_x
            + (target_x - init_x) * (i / HORIZON)
            + amplitude * np.sin(2 * np.pi * random_frequency * (i * TIME_INTERVAL))
        )
        traj_y.append(
            init_y
            + (target_y - init_y) * (i / HORIZON)
            + amplitude * np.sin(2 * np.pi * random_frequency * (i * TIME_INTERVAL))
        )
        traj_z.append(
            init_z
            + (target_z - init_z) * (i / HORIZON)
            + amplitude * np.sin(2 * np.pi * random_frequency * (i * TIME_INTERVAL))
        )

    # plot and save the generated trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(traj_x, traj_y, traj_z, label="Generated Trajectory")
    ax.scatter([init_x], [init_y], [init_z], color="green", label="Start", s=50)
    ax.scatter([target_x], [target_y], [target_z], color="red", label="Target", s=50)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Generated XYZ Trajectory")
    ax.legend()
    plt.savefig("generated_trajectory.svg")
    plt.close()

    print("Beginning maneuvers...")
    last_index = -1

    # MAIN LOOP WITH SAFETY CHECK
    while qcf.is_safe() and dt <= (MAX_TIME + TIME_MARGIN):

        # Terminate upon Esc command
        if last_key_pressed == pynput.keyboard.Key.esc:
            break
        print(last_key_pressed)

        # Mind the clock
        dt = time() - t

        # Take off and hover in the center of safe airspace for 5 seconds
        print(f"[t={int(dt)}] Maneuvering - Center...")

        # Set target
        time_index = int(dt / TIME_INTERVAL)

        # check if time index has skipped any steps
        if time_index > last_index + 1:
            print(f"[Warning]: Skipped from index {last_index} to {time_index}")
        last_index = time_index

        try:
            target = Pose(
                traj_x[time_index],
                traj_y[time_index],
                traj_z[time_index],
            )
        except:
            # Land
            while qcf.pose.z > 0.5:
                print(qcf.pose.z)
                print("landing...")
                qcf.land_in_place()

        # Engage
        qcf.safe_position_setpoint(target)
        # print decision-making time
        print(f"Decision-making time: {time() - dt} s")

    # Land
    while qcf.pose.z > 0.5:
        print(qcf.pose.z)
        print("landing...")
        qcf.land_in_place()
