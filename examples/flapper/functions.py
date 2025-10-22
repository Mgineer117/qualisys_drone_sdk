import random
from typing import Union
import numpy as np
import matplotlib.pyplot as plt

from cflib.crazyflie.log import LogConfig


def get_logger(group_list: list, TIME_INTERVAL: float) -> list:
    conf_list = []
    for group in group_list:
        logconf = LogConfig(name=group, period_in_ms=TIME_INTERVAL * 1000)
        # if group == "stabilizer":
        # logconf.add_variable("stabilizer.roll", "float")  # Same as stateEstimate.roll
        # logconf.add_variable("stabilizer.pitch", "float")  # Same as stateEstimate.pitch
        # logconf.add_variable("stabilizer.yaw", "float")  # Same as stateEstimate.yaw
        # logconf.add_variable("stabilizer.thrust", "float")  # Current thrust
        if group == "pos":
            logconf.add_variable("stateEstimate.x", "float")
            logconf.add_variable("stateEstimate.y", "float")
            logconf.add_variable("stateEstimate.z", "float")
        if group == "vel":
            logconf.add_variable("stateEstimate.vx", "float")
            logconf.add_variable("stateEstimate.vy", "float")
            logconf.add_variable("stateEstimate.vz", "float")
        if group == "acc":
            logconf.add_variable("stateEstimate.ax", "float")
            logconf.add_variable("stateEstimate.ay", "float")
            logconf.add_variable("stateEstimate.az", "float")
        if group == "motor":
            logconf.add_variable("motor.m1", "float")
            logconf.add_variable("motor.m2", "float")
            logconf.add_variable("motor.m3", "float")
            logconf.add_variable("motor.m4", "float")
        if group == "motor_req":
            logconf.add_variable("motor.m1req", "float")
            logconf.add_variable("motor.m2req", "float")
            logconf.add_variable("motor.m3req", "float")
            logconf.add_variable("motor.m4req", "float")
        if group == "gyro":
            logconf.add_variable("gyro.x", "float")
            logconf.add_variable("gyro.y", "float")
            logconf.add_variable("gyro.z", "float")
        if group == "target_pos":
            logconf.add_variable("ctrltarget.x", "float")
            logconf.add_variable("ctrltarget.y", "float")
            logconf.add_variable("ctrltarget.z", "float")
        if group == "target_vel":
            logconf.add_variable("ctrltarget.vx", "float")
            logconf.add_variable("ctrltarget.vy", "float")
            logconf.add_variable("ctrltarget.vz", "float")
        if group == "target_attitude":
            logconf.add_variable("ctrltarget.roll", "float")
            logconf.add_variable("ctrltarget.pitch", "float")
            logconf.add_variable("ctrltarget.yaw", "float")
        if group == "controller_cmd":
            logconf.add_variable("controller.cmd_thrust", "float")
            logconf.add_variable("controller.cmd_roll", "float")
            logconf.add_variable("controller.cmd_pitch", "float")
            logconf.add_variable("controller.cmd_yaw", "float")
        if group == "controller_attitude":
            logconf.add_variable("controller.roll", "float")
            logconf.add_variable("controller.pitch", "float")
            logconf.add_variable("controller.yaw", "float")
        if group == "controller_attitude_rate":
            logconf.add_variable("controller.rollRate", "float")
            logconf.add_variable("controller.pitchRate", "float")
            logconf.add_variable("controller.yawRate", "float")
        conf_list.append(logconf)

    return conf_list


def initialize_target_destination(init_x: int, init_y: int, init_z: int):
    # generate random x, y, delta z for specifying target
    delta_z = random.uniform(1.0, 3.0)

    target_x = init_x + random.uniform(-1.0, 1.0)
    target_y = init_y + random.uniform(-1.0, 1.0)
    target_z = init_z + delta_z

    return target_x, target_y, target_z


def plot_trajectory(traj_x: list, traj_y: list, traj_z: list):
    init_x, init_y, init_z = traj_x[0], traj_y[0], traj_z[0]
    target_x, target_y, target_z = traj_x[-1], traj_y[-1], traj_z[-1]

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


def generate_reference_trajectory(
    init_x: int,
    init_y: int,
    init_z: int,
    HORIZON: int,
    TIME_INTERVAL: float,
    visualize: bool = True,
    seed: int = None,
) -> Union[list, list, list]:

    # Set random seed for reproducibility
    seed = int(time()) if seed is None else seed
    random.seed(seed)
    np.random.seed(seed)

    # Sample random amplitude and frequency for sinusoidal perturbation
    amplitude = random.uniform(0.1, 0.3)
    random_frequency = random.uniform(0.1, 0.3)

    # Initialize target destination
    target_x, target_y, target_z = initialize_target_destination(init_x, init_y, init_z)

    # Generate trajectory with sinusoidal perturbation
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

    # Visualize trajectory if needed
    if visualize:
        plot_trajectory(traj_x, traj_y, traj_z)

    return traj_x, traj_y, traj_z, seed
