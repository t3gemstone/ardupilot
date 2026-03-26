#!/usr/bin/env python3
"""
SITL Swarm Test - launches N ArduPilot SITL instances and tests the swarm controller.
Run from the repo root: python3 gemstone/ai_swarm/sitl_test.py [--num 3]
Requires: mavproxy, ardupilot SITL binary in PATH or build/t3-gem-o1/bin/
"""

import argparse
import subprocess
import time
import sys
import os

ARDUPILOT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SITL_BIN       = os.path.join(ARDUPILOT_ROOT, "build/sitl/bin/arduplane")
SITL_VEHICLE   = os.environ.get("SITL_VEHICLE", "plane")
BASE_PORT      = 5760   # ports: 5760, 5770, 5780 ...
BASE_MAVLINK   = 14550  # UDP ports: 14550, 14560 ...


def launch_sitl(instance: int) -> list:
    port     = BASE_PORT + instance * 10
    out_port = BASE_MAVLINK + instance * 10
    
    # Instance 0 -> TCP:5760, UDP:14550
    # Instance 1 -> TCP:5770, UDP:14560
    
    # 1. Start ArduPlane
    plane_cmd = [
        SITL_BIN,
        "-I", str(instance),
        "--model", "plane",
        "--defaults", os.path.join(ARDUPILOT_ROOT, "Tools/autotest/models/plane.parm"),
        "--speedup", "1",
    ]
    plane_log = open(f"/tmp/ardupilot/arduplane_instance_{instance}.log", "w")
    print(f"[*] Starting ArduPlane Instance {instance} (TCP:{port})")
    plane_proc = subprocess.Popen(plane_cmd, stdout=plane_log, stderr=plane_log,
                                  stdin=subprocess.DEVNULL, cwd=ARDUPILOT_ROOT)
    
    # 2. Start MAVProxy to bridge SITL to the controller
    # MAVProxy connects to SITL via TCP (5760+I*10) and outputs to UDP (14550+I*10)
    mavproxy_path = "/home/vm/.local/bin/mavproxy.py"
    mav_cmd = [
        sys.executable, mavproxy_path,
        "--master", f"tcp:127.0.0.1:{port}",
        "--out",    f"udp:127.0.0.1:{out_port}",
    ]
    mav_log = open(f"/tmp/ardupilot/mavproxy_instance_{instance}.log", "w")
    print(f"[*] Starting MAVProxy Instance {instance} (Master:TCP:{port} -> Out:UDP:{out_port})")
    mav_proc = subprocess.Popen(mav_cmd, stdout=mav_log, stderr=mav_log,
                                stdin=subprocess.DEVNULL)
    
    return [plane_proc, mav_proc]


def launch_controller(drone_id: int, mode: str, broker: str, instance: int) -> subprocess.Popen:
    mavlink_url = f"udpin:127.0.0.1:{BASE_MAVLINK + instance * 10}"
    ctrl_script = os.path.join(os.path.dirname(__file__), "controller.py")
    cmd = [
        sys.executable, ctrl_script,
        "--id",      str(drone_id),
        "--mode",    mode,
        "--broker",  broker,
        "--mavlink", mavlink_url,
    ]
    log_file = open(f"/tmp/ardupilot/controller_drone_{drone_id}.log", "w")
    print(f"[*] Starting Controller Drone {drone_id} ({mode}) → {mavlink_url}")
    return subprocess.Popen(cmd, stdout=log_file, stderr=log_file,
                            stdin=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num",    type=int, default=3,          help="Number of drones (max 10)")
    parser.add_argument("--broker", default="localhost",           help="MQTT broker IP")
    parser.add_argument("--no-sitl", action="store_true",         help="Skip SITL, controllers only")
    args = parser.parse_args()

    num = min(args.num, 10)
    sitl_procs  = []
    ctrl_procs  = []

    os.makedirs("/tmp/ardupilot", exist_ok=True)

    # Pre-launch cleanup to avoid "Address already in use"
    print("[*] Performing pre-launch cleanup...")
    subprocess.run(["pkill", "-9", "-f", "arduplane"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "mavproxy"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "controller.py"], capture_output=True)
    time.sleep(1)

    try:
        if not args.no_sitl:
            print(f"\n[*] Launching {num} SITL instances...")
            for i in range(num):
                sitl_procs.extend(launch_sitl(i))
            print("[*] Waiting 15s for SITL to initialize...")
            time.sleep(15)

        print(f"\n[*] Launching {num} swarm controllers...")
        # Drone 1 = leader
        ctrl_procs.append(launch_controller(1, "leader", args.broker, 0))
        time.sleep(2)
        # Drones 2..N = followers
        for i in range(1, num):
            ctrl_procs.append(launch_controller(i + 1, "follower", args.broker, i))
            time.sleep(0.5)

        print(f"\n[+] Swarm of {num} drones running!")
        print("[*] Logs: /tmp/ardupilot/sitl_drone_*.log  /tmp/ardupilot/controller_drone_*.log")
        print("[*] Press Ctrl+C to stop all.\n")

        while True:
            time.sleep(5)

    except KeyboardInterrupt:
        print("\n[*] Stopping swarm...")
    finally:
        for p in ctrl_procs + sitl_procs:
            p.kill()
        print("[+] All processes stopped.")


if __name__ == "__main__":
    main()
