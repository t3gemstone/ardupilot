import time
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
conn.wait_heartbeat()
print(f"Connected to sysid {conn.target_system}")

msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
lat = msg.lat
lon = msg.lon

print("Arming and taking off via POSITION TARGET")
conn.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
time.sleep(1)
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)

# Try position target
conn.mav.set_position_target_global_int_send(
    0, 1, 1,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b0000111111111000,
    lat, lon, 100.0,
    0, 0, 0, 0, 0, 0, 0, 0
)

# And try DO_REPOSITION (often used to move) - wait, DO_REPOSITION is 192
conn.mav.command_int_send(
    1, 1, 0, mavutil.mavlink.MAV_CMD_DO_REPOSITION,
    0, 1, 
    -1, mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, float('nan'),
    lat, lon, 100.0
)

start = time.time()
while time.time() - start < 5:
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg: print(f"Alt: {msg.relative_alt / 1000.0}m")
    time.sleep(0.5)

