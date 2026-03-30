import os
import time
import json
from pymavlink import mavutil
from openai import OpenAI
import threading

# Configuration for Local LLM
OPENAI_API_BASE = os.getenv("OPENAI_API_BASE", "http://localhost:11435/v1")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "ollama")
MODEL_NAME = os.getenv("MODEL_NAME", "phi3:mini")

print(f"Connecting to Local LLM at {OPENAI_API_BASE} using model {MODEL_NAME}")
client = OpenAI(base_url=OPENAI_API_BASE, api_key=OPENAI_API_KEY)

# Connection to Drones
DRONES = {
    1: {"port": 14551, "name": "Leader", "conn": None, "state": {}},
    2: {"port": 14561, "name": "Follower-1", "conn": None, "state": {}},
    3: {"port": 14571, "name": "Follower-2", "conn": None, "state": {}},
    4: {"port": 14581, "name": "Follower-3", "conn": None, "state": {}},
}

def connect_drones():
    for sysid, info in DRONES.items():
        print(f"Connecting to {info['name']} on UDP:127.0.0.1:{info['port']}...")
        conn = mavutil.mavlink_connection(f'udpin:127.0.0.1:{info["port"]}')
        conn.wait_heartbeat()
        print(f"[{info['name']}] Heartbeat received!")
        info["conn"] = conn
        
        # Request data stream
        conn.mav.request_data_stream_send(sysid, conn.target_component, 
                                          mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
        
        # Set Q_GUIDED_MODE to 1 for VTOL hovering in GUIDED mode
        conn.mav.param_set_send(sysid, conn.target_component, b"Q_GUIDED_MODE", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        
        threading.Thread(target=read_telemetry, args=(sysid,), daemon=True).start()

def read_telemetry(sysid):
    conn = DRONES[sysid]["conn"]
    while True:
        msg = conn.recv_match(type=['GLOBAL_POSITION_INT', 'HEARTBEAT'], blocking=True)
        if not msg:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            DRONES[sysid]["state"]["alt"] = msg.relative_alt / 1000.0
            DRONES[sysid]["state"]["lat"] = msg.lat / 1e7
            DRONES[sysid]["state"]["lon"] = msg.lon / 1e7
        elif msg.get_type() == 'HEARTBEAT':
            DRONES[sysid]["state"]["mode"] = mavutil.mode_string_v10(msg)
            DRONES[sysid]["state"]["armed"] = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

def send_vtol_takeoff(sysid, alt):
    conn = DRONES[sysid]["conn"]
    if not conn: return
    # Plane GUIDED mode is 15
    conn.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
    time.sleep(0.5)
    # ARM
    conn.mav.command_long_send(sysid, conn.target_component,
                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    # NAV_TAKEOFF (22), Param7 = Altitude (QuadPlane will VTOL takeoff if Q_GUIDED_MODE=1)
    conn.mav.command_long_send(sysid, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    print(f"[{DRONES[sysid]['name']}] Armed and taking off vertically to {alt}m")

def send_formation_target(sysid, lat, lon, alt):
    conn = DRONES[sysid]["conn"]
    if not conn: return
    # Send GUIDED target
    conn.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15) # GUIDED
    conn.mav.set_position_target_global_int_send(
        0, sysid, conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), float(alt),
        0, 0, 0, 0, 0, 0, 0, 0
    )

def execute_macro(cmd_obj):
    cmd = cmd_obj.get("cmd")
    if cmd == "SWARM_TAKEOFF":
        alt = cmd_obj.get("alt", 20)
        # Takeoff all drones
        for sysid in DRONES:
            send_vtol_takeoff(sysid, alt)
            
        print(f"[Swarm] Waiting for drones to reach {alt}m...")
        target_alt = alt * 0.85 # Wait until 85% of target altitude is reached
        start_wait = time.time()
        while time.time() - start_wait < 45:
            all_reached = True
            for sysid in DRONES:
                curr_alt = DRONES[sysid]["state"].get("alt", 0)
                if curr_alt < target_alt:
                    all_reached = False
                    break
            if all_reached:
                print("[Swarm] All drones took off successfully!")
                break
            time.sleep(1)
            
    elif cmd == "V_FORMATION":
        print("[Swarm] Computing V-Formation offsets based on Leader...")
        leader_state = DRONES[1]["state"]
        if "lat" not in leader_state:
            print("No GPS lock on Leader yet!")
            return
            
        lat = leader_state["lat"]
        lon = leader_state["lon"]
        alt = leader_state.get("alt", 20)
        if alt < 5: alt = 50 # fallback secure alt
        
        # Offsets in degrees (approx)
        offsets = {
            2: (-0.0002, -0.0002), # Follower-1: back-left
            3: (-0.0002, 0.0002),  # Follower-2: back-right
            4: (-0.0004, 0),       # Follower-3: double-back center
        }
        for sysid, offset in offsets.items():
            t_lat = lat + offset[0]
            t_lon = lon + offset[1]
            send_formation_target(sysid, t_lat, t_lon, alt)
            print(f"[{DRONES[sysid]['name']}] Moving to V-position offset.")

def process_llm_llm(user_input):
    state_context = "Current Drone States:\n"
    for sysid, info in DRONES.items():
        state_context += f"Drone {sysid} ({info['name']}): Altitude {info['state'].get('alt', 0):.1f}m\n"

    system_prompt = f"""
You are an AI commanding a swarm of 4 drones.
{state_context}

Available Macros (JSON format output only!):
[
  {{"cmd": "SWARM_TAKEOFF", "alt": 100}},
  {{"cmd": "V_FORMATION"}}
]

User Command: "{user_input}"
Respond STRICTLY with the JSON array format, nothing else. Combine multiple commands in the array if needed.
"""
    try:
        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=[{"role": "user", "content": system_prompt}],
            temperature=0.0
        )
        actions_text = response.choices[0].message.content
        import re
        # Bulunduğu yerdeki tüm JSON objelerini ayıkla (phi3:mini'nin hatalı formatlarını tolere eder)
        matches = re.findall(r'\{[^{}]*\}', actions_text)
        
        valid_actions = 0
        for match in matches:
            try:
                act = json.loads(match)
                if "cmd" in act:
                    execute_macro(act)
                    valid_actions += 1
            except:
                pass
        
        if valid_actions == 0:
            print("LLM geçerli bir format döndüremedi. Gelen ham yanıt:\n", actions_text)
    except Exception as e:
        print("LLM Error:", e)

def main():
    print("Welcome to AI Drone Swarm CLI")
    try:
        connect_drones()
    except Exception as e:
        print("Could not connect to drones. Run 'task swarm-up' first.", e)
        return

    while True:
        try:
            cmd = input("\n[Swarm-CLI] Enter command (or 'exit'): ")
            if cmd.lower() in ['exit', 'quit']:
                break
            if cmd.strip():
                process_llm_llm(cmd)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
