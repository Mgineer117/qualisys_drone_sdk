import argparse
import json

import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
import math

# Load JSON file
with open('examples/flapper/circular_traj/circular_Z_20250423183719.json', 'r') as f:
    data = json.load(f)

# Plot positions
poses = data["pose"]
x_qual = [p[0] for p in poses]
y_qual = [p[1] for p in poses]
z_qual = [p[2] for p in poses]

targets = data["control"]
x_target = [p[0] for p in targets]
y_target = [p[1] for p in targets]
z_target = [p[2] for p in targets]

target_pos = data["target_pos"]
x_ctrltarget = [c["ctrltarget.x"] for c in target_pos]
y_ctrltarget = [c["ctrltarget.y"] for c in target_pos]
z_ctrltarget = [c["ctrltarget.z"] for c in target_pos]

position = data["pos"]
x_state = [p["stateEstimate.x"] for p in position]
y_state = [p["stateEstimate.y"] for p in position]
z_state = [p["stateEstimate.z"] for p in position]

fig, axs = plt.subplots(3, 1, figsize=(8, 12))
axs[0].plot(x_qual, label='x qualisys')
axs[0].plot(x_state, label='x state est')
axs[0].plot(x_target, label='x target')
axs[0].legend()
axs[1].plot(y_qual, label='y qualisys')
axs[1].plot(y_state, label='y state est')
axs[1].plot(y_target, label='y target')
axs[1].legend()
axs[2].plot(z_qual, label='z qualisys')
axs[2].plot(z_state, label='z state est')
axs[2].plot(z_target, label='z target')
axs[2].legend()
plt.show()

# Plot attitude
stabilizer = data["stabilizer"]
state_roll = [c["stabilizer.roll"] for c in stabilizer]
state_pitch = [c["stabilizer.pitch"] for c in stabilizer]
state_yaw = [c["stabilizer.yaw"] for c in stabilizer]

controller_attitude = data["controller_attitude"]
controller_roll = [c["controller.roll"] for c in controller_attitude]
controller_pitch = [c["controller.pitch"] for c in controller_attitude]
controller_yaw = [c["controller.yaw"] for c in controller_attitude]

fig, axs = plt.subplots(3, 1, figsize=(8, 12))
axs[0].plot(state_roll, label='roll state')
axs[0].plot(controller_roll, label='roll ctrl reference')
axs[0].legend()
axs[1].plot(state_pitch, label='pitch state')
axs[1].plot(controller_pitch, label='pitch ctrl reference')
axs[1].legend()
axs[2].plot(state_yaw, label='yaw state')
axs[2].plot(controller_yaw, label='yaw ctrl reference')
axs[2].legend()
plt.show()

# Plot attitude rates
attitude_rate = data["attitude_rate"] # [milliradians / sec]
roll_rate = [c["stateEstimateZ.rateRoll"]/1000*180/math.pi for c in attitude_rate]
pitch_rate = [c["stateEstimateZ.ratePitch"]/1000*180/math.pi for c in attitude_rate]
yaw_rate = [c["stateEstimateZ.rateYaw"]/1000*180/math.pi for c in attitude_rate]

controller_attitude_rate = data["controller_attitude_rate"]
controller_roll_rate = [c["controller.rollRate"] for c in controller_attitude_rate]
controller_pitch_rate = [c["controller.pitchRate"] for c in controller_attitude_rate]
controller_yaw_rate = [c["controller.yawRate"] for c in controller_attitude_rate]

# TODO: double check
fig, axs = plt.subplots(3, 1, figsize=(8, 12))
axs[0].plot(roll_rate, label='roll_rate state')
axs[0].plot(controller_roll_rate, label='roll_rate ctrl reference')
axs[0].legend()
axs[1].plot(pitch_rate, label='pitch_rate state')
axs[1].plot(controller_pitch_rate, label='pitch_rate ctrl reference')
axs[1].legend()
axs[2].plot(yaw_rate, label='yaw_rate state')
axs[2].plot(controller_yaw_rate, label='yaw_rate ctrl reference')
axs[2].legend()
plt.show()