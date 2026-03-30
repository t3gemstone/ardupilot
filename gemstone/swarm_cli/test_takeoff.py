import time
from pymavlink import mavutil

print("Connecting...")
conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
conn.wait_heartbeat()
print("Heartbeat received!")

# Check initial status
msg = conn.recv_match(type='HEARTBEAT', blocking=True)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
print(f"Initial mode: {mavutil.mode_string_v10(msg)}, Armed: {bool(armed)}")

conn.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
time.sleep(1)
msg = conn.recv_match(type='HEARTBEAT', blocking=True)
print(f"Mode after set_mode_send: {mavutil.mode_string_v10(msg)}")

# Send ARM command
print("Sending ARM Command...")
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# Check STATUSTEXT responses
start = time.time()
while time.time() - start < 3:
    msg = conn.recv_match(type='STATUSTEXT', blocking=False)
    if msg: print(f"STATUS: {msg.text}")
    time.sleep(0.1)

# Send VTOL_TAKEOFF command
print("Sending VTOL TAKEOFF Command to 100m...")
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 100) # Testing normal NAV_TAKEOFF
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 100) # Testing NAV_VTOL_TAKEOFF
start = time.time()
while time.time() - start < 3:
    msg = conn.recv_match(type='STATUSTEXT', blocking=False)
    if msg: print(f"STATUS: {msg.text}")
    msg = conn.recv_match(type='COMMAND_ACK', blocking=False)
    if msg: print(f"ACK: {msg}")
    time.sleep(0.1)
