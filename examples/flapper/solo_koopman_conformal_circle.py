"""
Updated Crazyflie circular trajectory using remote target computation API
"""

import argparse
from time import time

import numpy as np
from omegaconf import OmegaConf
from pydantic import BaseModel

from api.schema import TargetRequest
from api.service import get_target_position
from qfly import Pose, QualisysCrazyflie, World

# SETTINGS
cf_body_name = "flapper"
cf_uri = "radio://0/80/2M/E7E7E7E7E7"
cf_marker_ids = [1, 2, 3, 4]
qtm_ip = "128.174.245.190"


class FlapperConfig(BaseModel):
    trajectory_type: str  # XZ, XY, XYZ, XY2Z
    omega: float  # degrees per second
    radius: float  # meters
    takeoff_sec: float  # seconds
    tracking_sec: float  # seconds


def calc_target(config: FlapperConfig, t: float):
    theta = config.omega * t / 180 * np.pi
    if config.trajectory_type == "XZ":
        target_x = 0.5 * np.cos(theta)
        target_y = 0.0
        target_z = 0.5 * np.sin(theta) + 1.0
    elif config.trajectory_type == "XY":
        target_x = 2 * np.cos(theta)
        target_y = 2 * np.sin(theta)
        target_z = 1.0
    elif config.trajectory_type == "XYZ":
        target_x = 2 * np.cos(theta)
        target_y = 2 * np.sin(theta)
        target_z = config.radius * np.sin(theta) + 1.0
    elif config.trajectory_type == "XY2Z":
        target_x = 2 * np.cos(theta)
        target_y = 2 * np.sin(theta)
        target_z = 0.5 * np.sin(2 * theta) + 1.0
    return target_x, target_y, target_z


def main(config):
    world = World(expanse=1.8, speed_limit=1.1)
    # preflight check
    target_x, target_y, target_z = calc_target(config, 0)
    req = TargetRequest(
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
        cur_x=world.origin.x,
        cur_y=world.origin.y,
        cur_z=1.0,
        rot_mat=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
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
            t = time() - t0

            if t < config.takeoff_step:
                target = Pose(world.origin.x, world.origin.y, 1.0)
            elif t < 20:
                # fetch from remote service
                target_x, target_y, target_z = calc_target(config, t)
                req = TargetRequest(
                    target_x=target_x,
                    target_y=target_y,
                    target_z=target_z,
                    cur_x=qcf.pose.x,
                    cur_y=qcf.pose.y,
                    cur_z=qcf.pose.z,
                    rot_mat=qcf.rot_mat.tolist(),
                )
                target, status = get_target_position(req)
                if status != "OK":
                    print(f"Error: {status}")
                    break
            qcf.safe_position_setpoint(target)
            # log data
            data["pose"].append([qcf.pose.x, qcf.pose.y, qcf.pose.z])
            data["time"].append(t)
            data["control"].append([target.x, target.y, target.z])
        # Land
        first_z = qcf.pose.z
        landing_time = 5
        start_time = time()
        while qcf.is_safe():
            if time() - start_time < 5:
                print(f"[t={t}] Maneuvering - Center...")
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flapper Config")
    parser.add_argument(
        "config",
        type=str,
        help="Path to the config file",
    )
    args = parser.parse_args()

    # Load the configuration file
    cfg = OmegaConf.load(args.config)
    cfg_dict = OmegaConf.to_container(cfg, resolve=True)
    flapper_config = FlapperConfig(**cfg_dict)

    main(flapper_config)
