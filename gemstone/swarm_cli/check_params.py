import time
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
conn.wait_heartbeat()
print(f"Connected to sysid {conn.target_system}")

params_to_check = ['Q_ENABLE', 'Q_GUIDED_MODE', 'SYSID_THISMAV']

for param in params_to_check:
    print(f"Requesting {param}...")
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        param.encode('utf-8'),
        -1
    )
    start = time.time()
    while time.time() - start < 1:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=False)
        if msg:
            if msg.param_id == param:
                print(f"{param}: {msg.param_value}")
                break
