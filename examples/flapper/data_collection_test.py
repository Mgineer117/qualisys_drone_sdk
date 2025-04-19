"""
qfly | Qualisys Drone SDK Example Script: Solo Crazyflie

Takes off, flies circles around Z, Y, X axes.
ESC to land at any time.
"""

import json
from time import sleep, time
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import pynput
from qfly import Pose, QualisysCrazyflie, World, utils
from functools import partial

# SETTINGS
cf_body_name = "flapper"  # QTM rigid body name
cf_uri = "radio://0/80/2M/E7E7E7E701"  # Crazyflie address
cf_marker_ids = [1, 2, 3, 4]  # Active marker IDs
circle_radius = 0.5  # Radius of the circular flight path
circle_speed_factor = 0.12  # How fast the Crazyflie should move along circle
qtm_ip = "128.174.245.190"

last_saved_t = time()
save_freq = 0.1
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

# position, velocity, time, control
data = {}
# Listen to the keyboard
listener = pynput.keyboard.Listener(on_press=on_press)
listener.start()


# Set up world - the World object comes with sane defaults
world = World()

# Set up asyncronous logging configuration
conf_list = []
group_list = ["stabilizer", "pos", "vel", "acc", "motor", "motor_req", "gyro", "target"]
for group in group_list:
    logconf = LogConfig(name=group, period_in_ms=100)
    if group == "stabilizer":
        logconf.add_variable('stabilizer.roll', 'float')
        logconf.add_variable('stabilizer.pitch', 'float')
        logconf.add_variable('stabilizer.yaw', 'float')
        logconf.add_variable('stabilizer.thrust', 'float')
    if group == "pos":
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
    if group == "vel":
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('stateEstimate.vy', 'float')
        logconf.add_variable('stateEstimate.vz', 'float')
    if group == "acc":
        logconf.add_variable('stateEstimate.ax', 'float')
        logconf.add_variable('stateEstimate.ay', 'float')
        logconf.add_variable('stateEstimate.az', 'float')
    if group == "motor":    
        logconf.add_variable('motor.m1', 'float')
        logconf.add_variable('motor.m2', 'float')
        logconf.add_variable('motor.m3', 'float')
        logconf.add_variable('motor.m4', 'float')
    if group == "motor_req":
        logconf.add_variable('motor.m1req', 'float')
        logconf.add_variable('motor.m2req', 'float')
        logconf.add_variable('motor.m3req', 'float')
        logconf.add_variable('motor.m4req', 'float')
    if group == "gyro":
        logconf.add_variable('gyro.x', 'float')
        logconf.add_variable('gyro.y', 'float')
        logconf.add_variable('gyro.z', 'float')
    if group == "target":
        logconf.add_variable('ctrltarget.x', 'float')
        logconf.add_variable('ctrltarget.y', 'float')
        logconf.add_variable('ctrltarget.z', 'float')
    conf_list.append(logconf)

# Prepare for liftoff
with QualisysCrazyflie(
    cf_body_name, cf_uri, world, marker_ids=cf_marker_ids, qtm_ip=qtm_ip
) as qcf:

    # Let there be time
    t = time()
    dt = 0

    # Get PID gains
    #Set PID gains for x
    #print("PID position x Kp", qcf.cf.param.get_value('pid.position_p'))
    #print("PID position x Ki", qcf.cf.param.get_value('pid_position.x_ki'))
    #print("PID position x Kd", qcf.cf.param.get_value('pid_position.x_kd'))

    #Set PID gains for roll
    print("PID attitude roll Kp", qcf.cf.param.get_value('pid_attitude.roll_kp'))
    print("PID attitude roll Ki", qcf.cf.param.get_value('pid_attitude.roll_ki'))
    print("PID attitude roll Kd", qcf.cf.param.get_value('pid_attitude.roll_kd'))

    # Set PID gains for pitch
    print("PID attitude pitch Kp", qcf.cf.param.get_value('pid_attitude.pitch_kp'))
    print("PID attitude pitch Ki", qcf.cf.param.get_value('pid_attitude.pitch_ki'))
    print("PID attitude pitch Kd", qcf.cf.param.get_value('pid_attitude.pitch_kd'))

    # Set PID gains for yaw
    print("PID attitude yaw Kp", qcf.cf.param.get_value('pid_attitude.yaw_kp'))
    print("PID attitude yaw Ki", qcf.cf.param.get_value('pid_attitude.yaw_ki'))
    print("PID attitude yaw Kd", qcf.cf.param.get_value('pid_attitude.yaw_kd'))
    
    for group in group_list:
        data[group] = []
    data["time"] = []
    for logconf in conf_list:
        qcf.cf.log.add_config(logconf)
    for group, logconf in zip(group_list, conf_list):
        callback = partial(log_callback, data_log=data, key=group)
        logconf.data_received_cb.add_callback(callback)
        logconf.start()
    # MAIN LOOP WITH SAFETY CHECK
    while qcf.is_safe():

        # Terminate upon Esc command
        if last_key_pressed == pynput.keyboard.Key.esc:
            break
        # Mind the clock
        dt = time() - t
        # Calculate Crazyflie's angular position in circle, based on time
        phi = circle_speed_factor * dt * 360
        if dt < 3:
            if time() - last_saved_t > save_freq:
                data["time"].append(time())
                last_saved_t = time()
                continue
        else:
            for logconf in conf_list:
                logconf.stop()
            break
    
# Open a file in write mode and use json.dump() to write the dictionary to the file
with open("data.json", "w") as file:
    json.dump(data, file, indent=4)

print("Dictionary has been saved to data.json")
