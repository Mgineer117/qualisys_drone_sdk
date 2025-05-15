from typing import List

from pydantic import BaseModel, ConfigDict


class TargetRequest(BaseModel):
    dt: float
    axis: str
    radius: float
    speed: float
    origin_x: float
    origin_y: float
    origin_z: float
    x_cur: float
    y_cur: float
    z_cur: float
    rot_mat: List[List[float]]


class ControlStatus(str):
    OK = "OK"
    ERROR = "ERROR"


class TargetResponse(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    x: float
    y: float
    z: float
    yaw: float
    status: ControlStatus
