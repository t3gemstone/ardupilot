import time
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
conn.wait_heartbeat()
print("Heartbeat")

print("Setting Q_GUIDED_MODE=1...")
conn.mav.param_set_send(
    1, 1,
    b"Q_GUIDED_MODE",
    1.0,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)
time.sleep(1)

conn.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
time.sleep(1)
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 100)

start = time.time()
while time.time() - start < 10:
    msg = conn.recv_match(type=['GLOBAL_POSITION_INT', 'STATUSTEXT'], blocking=False)
    if msg:
        if msg.get_type() == 'STATUSTEXT':
            print(f"STATUS: {msg.text}")
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            print(f"Alt: {msg.relative_alt / 1000.0}m")
    time.sleep(0.1)
