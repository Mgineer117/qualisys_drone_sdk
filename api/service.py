from typing import List

import requests

from api.schema import TargetRequest, TargetResponse
from qfly import Pose

# Configure your server URL
SERVER_URL = "http://localhost:8000"


def get_target_position(
    dt: float,
    axis: str,
    radius: float,
    speed: float,
    origin_x: float,
    origin_y: float,
    origin_z: float,
    x_cur: float,
    y_cur: float,
    z_cur: float,
    rot_mat: List[List[float]],
) -> Pose:
    req = TargetRequest(
        dt=dt,
        axis=axis,
        radius=radius,
        speed=speed,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_z=origin_z,
        x_cur=x_cur,
        y_cur=y_cur,
        z_cur=z_cur,
        rot_mat=rot_mat,
    )
    resp = requests.post(f"{SERVER_URL}/compute_target", json=req.model_dump())
    resp.raise_for_status()
    data = TargetResponse(**resp.json())

    if data.status != TargetResponse.status.OK:
        return Pose(x_cur, y_cur, z_cur, yaw=0.0), data.status
    else:
        return Pose(data.x, data.y, data.z, yaw=data.yaw), data.status
