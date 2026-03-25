import argparse
import json
import time
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from pymavlink import mavutil
import ollama

# --- Configuration ---
DEFAULT_MQTT_BROKER = "localhost"
DEFAULT_MQTT_PORT = 1883
DEFAULT_MAVLINK_URL = "udpin:0.0.0.0:14550"
LEADER_MODEL = "phi3:mini"
FOLLOWER_MODEL = "qwen2:0.5b"
STT_MODEL = "tiny"

NATO_ALPHABET = {
    1: "Alpha", 2: "Bravo", 3: "Charlie", 4: "Delta", 5: "Echo",
    6: "Foxtrot", 7: "Golf", 8: "Hotel", 9: "India", 10: "Juliett"
}
ID_TO_NATO = {v: k for k, v in NATO_ALPHABET.items()}

class SwarmAIController:
    def __init__(self, drone_id, mode="follower", broker=DEFAULT_MQTT_BROKER, mavlink_url=DEFAULT_MAVLINK_URL):
        self.drone_id = drone_id
        self.mode = mode
        self.broker = broker
        self.mavlink_url = mavlink_url
        self.client = mqtt.Client(CallbackAPIVersion.VERSION2, f"drone_{drone_id}")
        self.swarm_status = {} # ID: {pos: (lat, lon), batt: 100}
        
        # Connect to ArduPilot (non-blocking with timeout)
        print(f"[*] Connecting to ArduPilot via {mavlink_url}...")
        self.master = mavutil.mavlink_connection(mavlink_url)
        hb = self.master.wait_heartbeat(timeout=10)
        if hb:
            print(f"[+] Heartbeat from system {self.master.target_system} received!")
        else:
            print(f"[!] No heartbeat yet — continuing without confirmed ArduPilot connection.")

    def on_connect(self, client, userdata, flags, rc):
        print(f"[*] Connected to MQTT Broker with result code {rc}")
        client.subscribe("swarm/telemetry")
        if self.mode == "follower" or self.mode == "all":
            client.subscribe("swarm/commands")
            client.subscribe(f"swarm/drone/{self.drone_id}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        
        if topic == "swarm/telemetry":
            data = json.loads(payload)
            self.swarm_status[data['id']] = data
        elif topic.startswith("swarm/"):
            print(f"[#] Received command: {payload}")
            self.process_swarm_command(payload)

    def solve_assignment(self, num_drones, target_pos):
        """Finds N drones closest to target_pos (The Assignment Problem)."""
        distances = []
        for d_id, status in self.swarm_status.items():
            pos = status.get('pos')
            if pos:
                dist = ((pos[0]-target_pos[0])**2 + (pos[1]-target_pos[1])**2)**0.5
                distances.append((d_id, dist))
        
        distances.sort(key=lambda x: x[1])
        return [d[0] for d in distances[:num_drones]]

    def run_leader_cli(self):
        """Leader loop: takes text input."""
        print(f"\n[LEADER MODE] Drone {self.drone_id} ({NATO_ALPHABET[self.drone_id]}) active.")
        while True:
            cmd = input("Swarm Command > ")
            
            if cmd.lower() in ["exit", "quit"]: break
            
            # Mission Planning Prompt (Expert Enhanced)
            prompt = f"""
            You are the Expert Swarm Leader AI. 
            Drones Available: {json.dumps(self.swarm_status)}
            Task: {cmd}
            Respond with a swarm directive string. 
            If the task requires a subset of drones, specify the count (e.g., 'Alpha Team (3 drones) move to...').
            """
            response = ollama.generate(model=LEADER_MODEL, prompt=prompt)
            directive = response['response']
            print(f"[*] Directive: {directive}")
            self.client.publish("swarm/commands", directive)

    def process_swarm_command(self, command_text):
        """Uses local LLM to translate swarm command to MAVLink."""
        prompt = f"""
        You are an ArduPilot Expert AI. Translate the following swarm command into a specific ArduPilot action.
        Command: {command_text}
        Respond ONLY with a JSON object in this format:
        {{
            "action": "TAKEOFF" | "GOTO" | "LAND" | "ARM" | "DISARM",
            "params": {{ "alt": 10, "lat": 0, "lon": 0 }}
        }}
        """
        try:
            response = ollama.generate(model=FOLLOWER_MODEL, prompt=prompt)
            action_data = json.loads(response['response'])
            self.execute_action(action_data)
        except Exception as e:
            print(f"[!] Error processing LLM command: {e}")

    def execute_action(self, action_data):
        action = action_data.get("action")
        params = action_data.get("params", {})
        
        print(f"[*] Executing ArduPilot Action: {action} with {params}")
        
        if action == "ARM":
            self.master.arducopter_arm()
        elif action == "TAKEOFF":
            alt = params.get("alt", 10)
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
        elif action == "LAND":
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    def start(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker, DEFAULT_MQTT_PORT, 60)
        
        if self.mode == "leader":
            self.client.loop_start()
            self.run_leader_cli()
        else:
            print(f"[*] Follower mode active on Drone {self.drone_id}. Waiting for commands...")
            self.client.loop_forever()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI Swarm Controller for ArduPilot")
    parser.add_argument("--id", type=int, required=True, help="Unique Drone ID")
    parser.add_argument("--mode", choices=["leader", "follower"], default="follower")
    parser.add_argument("--broker", default=DEFAULT_MQTT_BROKER)
    parser.add_argument("--mavlink", default=DEFAULT_MAVLINK_URL)
    args = parser.parse_args()

    controller = SwarmAIController(args.id, args.mode, args.broker, args.mavlink)
    controller.start()
