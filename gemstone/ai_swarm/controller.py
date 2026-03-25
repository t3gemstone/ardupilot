#!/usr/bin/env python3
"""
AI Swarm Controller for ArduPilot on T3-GEM-O1
Supports: async MAVLink, MQTT peer communication, decentralized bidding, telemetry publishing.
Usage:
  Leader:   python3 controller.py --id 1 --mode leader
  Follower: python3 controller.py --id 2 --mode follower --broker 192.168.x.x
"""

import argparse
import json
import logging
import struct
import threading
import time

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from pymavlink import mavutil
import ollama

# --- Configuration ---
DEFAULT_MQTT_BROKER   = "localhost"
DEFAULT_MQTT_PORT     = 1883
DEFAULT_MAVLINK_URL   = "udpin:0.0.0.0:14550"
LEADER_MODEL          = "phi3:mini"
FOLLOWER_MODEL        = "qwen2:0.5b"
TELEMETRY_INTERVAL    = 2.0   # seconds between telemetry publishes
HEARTBEAT_TIMEOUT     = 10    # seconds

NATO_ALPHABET = {
    1: "Alpha",  2: "Bravo",   3: "Charlie", 4: "Delta",  5: "Echo",
    6: "Foxtrot",7: "Golf",    8: "Hotel",   9: "India",  10: "Juliett"
}

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("swarm")


class SwarmAIController:
    def __init__(self, drone_id: int, mode: str, broker: str, mavlink_url: str):
        self.drone_id    = drone_id
        self.mode        = mode
        self.broker      = broker
        self.mavlink_url = mavlink_url
        self.nato_name   = NATO_ALPHABET.get(drone_id, f"Drone{drone_id}")

        # State
        self.swarm_status: dict = {}   # {id: {pos, batt, status}}
        self.active_bids:  dict = {}   # {task_id: [bids]}
        self.my_telemetry: dict = {"id": drone_id, "pos": [0.0, 0.0, 0.0],
                                   "batt": 100, "status": "idle"}
        self._lock = threading.Lock()
        self._running = True

        # MQTT setup (Paho v2 API)
        self.client = mqtt.Client(
            CallbackAPIVersion.VERSION2,
            client_id=f"drone_{drone_id}",
            protocol=mqtt.MQTTv5
        )
        self.client.on_connect    = self._on_connect
        self.client.on_message    = self._on_message
        self.client.on_disconnect = self._on_disconnect

        # MAVLink — async, non-blocking
        log.info(f"Connecting to ArduPilot via {mavlink_url}...")
        self.master = mavutil.mavlink_connection(mavlink_url)
        hb = self.master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
        if hb:
            log.info(f"Heartbeat OK — sys {self.master.target_system}")
        else:
            log.warning("No heartbeat yet — continuing without ArduPilot confirmation.")

    # ─── MQTT Callbacks ───────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        log.info(f"MQTT connected (rc={reason_code})")
        client.subscribe("swarm/telemetry")
        client.subscribe("swarm/commands")
        client.subscribe("swarm/bid/response")
        client.subscribe(f"swarm/drone/{self.drone_id}")
        client.subscribe(f"swarm/bid/task/{self.drone_id}")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        log.warning(f"MQTT disconnected (rc={reason_code}), reconnecting...")

    def _on_message(self, client, userdata, msg):
        topic   = msg.topic
        payload = msg.payload.decode()
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            data = payload  # plain text command

        if topic == "swarm/telemetry":
            with self._lock:
                self.swarm_status[data["id"]] = data

        elif topic == "swarm/commands" or topic == f"swarm/drone/{self.drone_id}":
            log.info(f"Command received: {data}")
            threading.Thread(target=self._process_command, args=(data,), daemon=True).start()

        elif topic == "swarm/bid/response":
            with self._lock:
                task_id = data.get("task_id")
                if task_id not in self.active_bids:
                    self.active_bids[task_id] = []
                self.active_bids[task_id].append(data)

        elif topic == f"swarm/bid/task/{self.drone_id}":
            # Follower received a final task assignment
            threading.Thread(target=self._process_command, args=(data,), daemon=True).start()

    # ─── Telemetry ────────────────────────────────────────────────────────────

    def _telemetry_loop(self):
        """Continuously publish own telemetry and read MAVLink messages."""
        while self._running:
            # Read MAVLink in background
            try:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    if msg.get_type() == "GLOBAL_POSITION_INT":
                        with self._lock:
                            self.my_telemetry["pos"] = [
                                msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3
                            ]
                    elif msg.get_type() == "BATTERY_STATUS":
                        with self._lock:
                            self.my_telemetry["batt"] = msg.battery_remaining
            except Exception:
                pass

            # Publish telemetry
            with self._lock:
                payload = json.dumps(self.my_telemetry)
            self.client.publish("swarm/telemetry", payload, qos=0, retain=False)
            time.sleep(TELEMETRY_INTERVAL)

    # ─── Smart Assignment (Squared-distance Auction) ──────────────────────────

    def solve_assignment(self, num_drones: int, target_pos: list) -> list:
        """Select the N drones closest to target_pos."""
        with self._lock:
            candidates = [
                (did, s["pos"]) for did, s in self.swarm_status.items()
                if s.get("pos")
            ]
        ranked = sorted(
            candidates,
            key=lambda x: sum((x[1][i] - target_pos[i])**2 for i in range(2))
        )
        return [d[0] for d in ranked[:num_drones]]

    def _issue_bid_request(self, task_id: str, task_text: str, num_needed: int):
        """Leader broadcasts a bid request; followers respond with their score."""
        payload = json.dumps({
            "task_id":    task_id,
            "task":       task_text,
            "num_needed": num_needed
        })
        self.client.publish("swarm/commands", payload)
        time.sleep(1.0)  # collect bids
        with self._lock:
            bids = sorted(
                self.active_bids.get(task_id, []),
                key=lambda b: b.get("score", 9999)
            )
        winners = [b["drone_id"] for b in bids[:num_needed]]
        log.info(f"Task {task_id}: assigned to {winners}")
        for w in winners:
            self.client.publish(f"swarm/bid/task/{w}", json.dumps({
                "task_id": task_id,
                "action":  task_text
            }))
        return winners

    def _submit_bid(self, task_id: str, task_text: str):
        """Follower computes bid score (lower = more capable) and responds."""
        with self._lock:
            batt = self.my_telemetry["batt"]
        score = 100 - batt  # higher battery → lower score → preferred
        self.client.publish("swarm/bid/response", json.dumps({
            "task_id":  task_id,
            "drone_id": self.drone_id,
            "score":    score
        }))

    # ─── Command Execution (Follower LLM → MAVLink) ──────────────────────────

    def _process_command(self, command_data):
        """Translate command text to MAVLink via local LLM."""
        if isinstance(command_data, dict):
            # Bidding-style: may contain a bid request for follower
            if "num_needed" in command_data:
                self._submit_bid(command_data["task_id"], command_data["task"])
                return
            cmd_text = command_data.get("action", str(command_data))
        else:
            cmd_text = command_data

        prompt = f"""You are an ArduPilot Expert. Convert this swarm command to a single MAVLink action.
Command: {cmd_text}
Respond ONLY with valid JSON:
{{
  "action": "ARM"|"DISARM"|"TAKEOFF"|"LAND"|"GOTO"|"SET_MODE",
  "params": {{"alt": 10, "lat": 0.0, "lon": 0.0, "mode": "GUIDED"}}
}}"""
        try:
            resp = ollama.generate(model=FOLLOWER_MODEL, prompt=prompt, options={"temperature": 0.1})
            raw = resp["response"].strip()
            # Extract JSON block if mixed with text
            start = raw.find("{")
            end   = raw.rfind("}") + 1
            action_data = json.loads(raw[start:end])
            self._execute_action(action_data)
        except Exception as e:
            log.error(f"LLM command error: {e}")

    def _execute_action(self, action_data: dict):
        action = action_data.get("action", "").upper()
        params = action_data.get("params", {})
        log.info(f"Executing {action} {params}")

        if action == "ARM":
            self.master.arducopter_arm()

        elif action == "DISARM":
            self.master.arducopter_disarm()

        elif action == "SET_MODE":
            mode_str = params.get("mode", "GUIDED")
            mode_id = self.master.mode_mapping().get(mode_str)
            if mode_id is not None:
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)

        elif action == "TAKEOFF":
            alt = float(params.get("alt", 10))
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, alt)

        elif action == "LAND":
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0)

        elif action == "GOTO":
            lat = int(float(params.get("lat", 0)) * 1e7)
            lon = int(float(params.get("lon", 0)) * 1e7)
            alt = float(params.get("alt", 10))
            self.master.mav.set_position_target_global_int_send(
                0,
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,   # position only
                lat, lon, alt,
                0, 0, 0, 0, 0, 0, 0, 0)

    # ─── Leader CLI ───────────────────────────────────────────────────────────

    def _leader_cli(self):
        log.info(f"\n[LEADER] {self.nato_name} (Drone {self.drone_id}) ready. Type 'exit' to quit.\n")
        while self._running:
            try:
                cmd = input("Swarm > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            if cmd.lower() in ("exit", "quit"):
                break

            # Leader LLM: decompose command into a swarm directive
            online = list(self.swarm_status.keys())
            prompt = f"""You are an Expert Swarm Leader AI managing {len(online)+1} drones.
Online followers: {online}
High-level task: "{cmd}"

Generate a concise swarm directive string and estimate how many drones are needed.
Respond ONLY with JSON:
{{
  "directive": "...",
  "num_drones": 3,
  "broadcast": true
}}"""
            try:
                resp = ollama.generate(model=LEADER_MODEL, prompt=prompt, options={"temperature": 0.2})
                raw = resp["response"].strip()
                start, end = raw.find("{"), raw.rfind("}")+1
                plan = json.loads(raw[start:end])
            except Exception as e:
                log.error(f"Leader LLM error: {e}")
                continue

            directive  = plan.get("directive", cmd)
            num_needed = int(plan.get("num_drones", len(online) or 1))
            log.info(f"Directive: {directive} | Drones needed: {num_needed}")

            if plan.get("broadcast", True):
                self.client.publish("swarm/commands", json.dumps({
                    "task_id":    f"task_{int(time.time())}",
                    "task":       directive,
                    "num_needed": num_needed
                }))
            else:
                assigned = self.solve_assignment(num_needed, [0, 0])
                for did in assigned:
                    self.client.publish(f"swarm/drone/{did}", json.dumps({"action": directive}))

    # ─── Start ────────────────────────────────────────────────────────────────

    def start(self):
        self.client.connect(self.broker, DEFAULT_MQTT_PORT, keepalive=60)
        self.client.loop_start()

        # Start async telemetry + MAVLink reader
        t = threading.Thread(target=self._telemetry_loop, daemon=True)
        t.start()

        if self.mode == "leader":
            self._leader_cli()
        else:
            log.info(f"[FOLLOWER] {self.nato_name} (Drone {self.drone_id}) listening...")
            try:
                while self._running:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass

        self._running = False
        self.client.loop_stop()
        self.client.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI Swarm Controller for ArduPilot")
    parser.add_argument("--id",      type=int,  required=True,            help="Drone ID (1-10)")
    parser.add_argument("--mode",    choices=["leader", "follower"],       default="follower")
    parser.add_argument("--broker",  default=DEFAULT_MQTT_BROKER,          help="MQTT Broker IP")
    parser.add_argument("--mavlink", default=DEFAULT_MAVLINK_URL,          help="MAVLink connection string")
    args = parser.parse_args()

    ctrl = SwarmAIController(args.id, args.mode, args.broker, args.mavlink)
    ctrl.start()
