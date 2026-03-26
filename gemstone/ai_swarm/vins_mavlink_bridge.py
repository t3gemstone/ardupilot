#!/usr/bin/env python3
"""
VINS-Fusion → ArduPilot MAVLink Bridge
Subscribes to VINS-Fusion's pose output (/vins_estimator/odometry) via ROS
and sends VISION_POSITION_ESTIMATE messages to ArduPilot EKF3.

Usage:
  python3 vins_mavlink_bridge.py --ardupilot 192.168.7.2:14550

ArduPilot parameters to set:
  AHRS_EKF_TYPE = 3         (use EKF3)
  EK3_SRC1_POSXY = 6       (ExternalNav)
  EK3_SRC1_VELXY = 6       (ExternalNav)
  EK3_SRC1_POSZ  = 1       (Baro)
  VISO_TYPE       = 1       (Enable external vision)
"""

import argparse
import math
import sys
import time
import threading

from pymavlink import mavutil

# Optional ROS import — graceful fallback for dev/test without ROS
try:
    import rospy
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class VINSMAVLinkBridge:
    def __init__(self, ardupilot_addr: str):
        host, _, port = ardupilot_addr.partition(":")
        port = int(port) if port else 14550
        conn_str = f"udpout:{host}:{port}"

        print(f"[*] Connecting to ArduPilot → {conn_str}")
        self.mav = mavutil.mavlink_connection(conn_str, source_system=200)

        self._pose = None
        self._lock = threading.Lock()
        self._running = True

        # Send heartbeat so ArduPilot recognises the companion computer
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    # === Heartbeat ===========================================================

    def _heartbeat_loop(self):
        while self._running:
            self.mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(1.0)

    # === Send Vision Pose to ArduPilot =======================================

    def send_vision_position(self, x: float, y: float, z: float,
                              roll: float, pitch: float, yaw: float,
                              confidence: float = 90.0):
        """Send VISION_POSITION_ESTIMATE to ArduPilot EKF3."""
        usec = int(time.time() * 1e6)
        self.mav.mav.vision_position_estimate_send(
            usec,
            x, y, z,
            roll, pitch, yaw
        )

    def send_vision_speed(self, vx: float, vy: float, vz: float):
        """Send VISION_SPEED_ESTIMATE for velocity fusion."""
        usec = int(time.time() * 1e6)
        self.mav.mav.vision_speed_estimate_send(usec, vx, vy, vz)

    # === ROS Callback ========================================================

    def _odom_callback(self, msg: "Odometry"):
        """Called for every VINS odometry message."""
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        v  = msg.twist.twist.linear

        # Quaternion → Euler
        roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)

        self.send_vision_position(p.x, p.y, p.z, roll, pitch, yaw)
        self.send_vision_speed(v.x, v.y, v.z)

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        """Convert quaternion to (roll, pitch, yaw) in radians."""
        sinr = 2 * (w*x + y*z)
        cosr = 1 - 2*(x*x + y*y)
        roll = math.atan2(sinr, cosr)

        sinp = 2*(w*y - z*x)
        pitch = math.asin(max(-1, min(1, sinp)))

        siny = 2*(w*z + x*y)
        cosy = 1 - 2*(y*y + z*z)
        yaw = math.atan2(siny, cosy)

        return roll, pitch, yaw

    # === Simulation Mode (no ROS) ============================================

    def _sim_loop(self):
        """Send dummy pose for testing without ROS/VINS."""
        print("[!] ROS not available — running in simulation mode (dummy pose).")
        t = 0.0
        while self._running:
            # Slowly rotating in place for demo
            self.send_vision_position(0.0, 0.0, -1.5, 0.0, 0.0, t * 0.1)
            self.send_vision_speed(0.0, 0.0, 0.0)
            t += 0.1
            time.sleep(0.1)  # 10 Hz

    # === Start ================================================================

    def start(self):
        if ROS_AVAILABLE:
            rospy.init_node("vins_mavlink_bridge", anonymous=True)
            rospy.Subscriber("/vins_estimator/odometry", Odometry, self._odom_callback)
            print("[+] Subscribed to /vins_estimator/odometry @ 10 Hz")
            print("[+] Bridge running — sending VISION_POSITION_ESTIMATE to ArduPilot")
            try:
                rospy.spin()
            except rospy.ROSInterruptException:
                pass
        else:
            self._sim_loop()

        self._running = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VINS-Fusion → ArduPilot MAVLink Bridge")
    parser.add_argument(
        "--ardupilot",
        default="192.168.7.2:14550",
        help="ArduPilot IP:PORT (default: 192.168.7.2:14550)"
    )
    args = parser.parse_args()

    bridge = VINSMAVLinkBridge(args.ardupilot)
    bridge.start()
