#!/usr/bin/env python3
"""Sample for ARM_DRIVER_INTERFACE.md: the topics of interest on a CloudChef rig.

    source <the setup file CloudChef gives you for the rig>
    python3 arm_driver_sample.py                      # read only, prints for 5 s
    python3 arm_driver_sample.py --arm follower_left  # the other arm
    python3 arm_driver_sample.py --wiggle             # ALSO moves joint 5 by 0.05 rad and back

Standard library + rclpy only. Reading is always safe. --wiggle commands the
arm: run it only with the CloudChef operator at the emergency stop.

Things to know before commanding:
  * The arm holds the LAST target it received, forever. There is no timeout.
  * Nothing smooths your targets. Ramp them yourself.
  * One writer per arm. This script refuses to command if someone else is.
  * Make your first target equal to the measured position.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray

p = argparse.ArgumentParser()
p.add_argument("--arm", default="follower_right", choices=["follower_left", "follower_right"])
p.add_argument("--wiggle", action="store_true", help="move joint 5 by +0.05 rad and back")
args = p.parse_args()

ARM = "/components/" + args.arm

# ---- what you read -------------------------------------------------------
# Arm, 200 Hz. 7 values: joint_0..joint_5 in rad, then the gripper
# (left_carriage_joint, 0 = closed .. 1 = open). effort is motor torque, N*m.
JOINT_STATES = ARM + "/joint_states"                    # sensor_msgs/JointState
EFFORTS = ARM + "/external_efforts"                     # Float64MultiArray, same 7 efforts
TEMPERATURES = ARM + "/joint_temperatures"              # Float64MultiArray, deg C, ~2 Hz
LIMITS = ARM + "/reference_controller/limits"           # Float64MultiArray, latched:
#                                                         6 lower limits then 6 upper, rad
# Cameras, JPEG. Topic names differ per rig; these are a ZED rig's. Use
# header.stamp (capture time), not your receive time.
CAMERAS = {
    "overhead": "/components/cam_overhead/color/image_rect_raw/compressed",
    "wrist_left": "/components/cam_wrist_left/color/image_raw/compressed",
    "wrist_right": "/components/cam_wrist_right/color/image_raw/compressed",
}

# ---- what you write ------------------------------------------------------
# 6 joint targets, rad, order joint_0..joint_5. Reliable, queue depth 1.
POSITION_CMD = ARM + "/forward_position_controller/commands"   # Float64MultiArray
GRIPPER_CMD = ARM + "/gripper_forward_controller/commands"     # Float64MultiArray, 1 value 0..1
# Also available, not used here (these need the ccros_msgs types):
#   ARM + "/reference_controller/commands"   ccros_msgs/msg/JointReference (impedance)
#   ARM + "/set_arm_gains"                   ccros_msgs/srv/SetArmGains    (stiffness)
#   ARM + "/set_control_mode"                ccros_msgs/srv/SetControlMode ("position" | "zero_torque")
#   ARM + "/follow_joint_trajectory"         control_msgs/action/FollowJointTrajectory

rclpy.init()
node = rclpy.create_node("arm_driver_sample")
seen = {}


def record(key, summary):
    def cb(msg):
        d = seen.setdefault(key, {"n": 0})
        d["n"] += 1
        d["last"] = summary(msg)
        d["msg"] = msg
    return cb


def rounded(values, digits=3):
    return [round(v, digits) for v in values]


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(JointState, JOINT_STATES, record(
    "joint_states", lambda m: "pos %s  effort %s" % (rounded(m.position), rounded(m.effort, 2))),
    qos_profile_sensor_data)
node.create_subscription(Float64MultiArray, EFFORTS, record(
    "external_efforts", lambda m: str(rounded(m.data, 2))), qos_profile_sensor_data)
node.create_subscription(Float64MultiArray, TEMPERATURES, record(
    "joint_temperatures", lambda m: str(rounded(m.data, 1))), qos_profile_sensor_data)
node.create_subscription(Float64MultiArray, LIMITS, record(
    "limits", lambda m: "lower %s upper %s" % (rounded(m.data[:6], 2), rounded(m.data[6:], 2))), LATCHED)
for name, topic in CAMERAS.items():
    node.create_subscription(CompressedImage, topic, record(
        "cam_" + name, lambda m: "%s, %d bytes, stamp %d.%09d" % (
            m.format, len(m.data), m.header.stamp.sec, m.header.stamp.nanosec)),
        qos_profile_sensor_data)


def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.003)


WINDOW = 5.0
spin(WINDOW)
print("=== %s, %.0f s ===" % (args.arm, WINDOW))
for key in ["joint_states", "external_efforts", "joint_temperatures", "limits"] + ["cam_" + c for c in CAMERAS]:
    d = seen.get(key)
    print("%-20s %s" % (key, "NOTHING RECEIVED" if d is None else "%5.1f Hz  %s" % (d["n"] / WINDOW, d["last"])))
if "joint_states" in seen:
    print("joint names:", list(seen["joint_states"]["msg"].name))

if not args.wiggle:
    sys.exit(0)

# ---- a small, slow, single-joint move -------------------------------------
if "joint_states" not in seen:
    sys.exit("no joint state; not commanding")
if node.count_publishers(POSITION_CMD):
    sys.exit("another node is already commanding this arm; not commanding")

JOINT, AMPLITUDE = 5, 0.05                      # wrist roll, rad
start = list(seen["joint_states"]["msg"].position[:6])
pub = node.create_publisher(Float64MultiArray, POSITION_CMD, QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
spin(0.5)                                        # let the driver match


def offset(t):                                   # 1 s out, 1 s hold, 1 s back
    if t < 1.0:
        return AMPLITUDE * 0.5 * (1 - math.cos(math.pi * t))
    if t < 2.0:
        return AMPLITUDE
    if t < 3.0:
        return AMPLITUDE * 0.5 * (1 + math.cos(math.pi * (t - 2.0)))
    return 0.0


t0 = time.time()
peak = 0.0
while time.time() - t0 < 3.5:
    target = list(start)
    target[JOINT] += offset(time.time() - t0)
    pub.publish(Float64MultiArray(data=target))  # ~100 Hz
    spin(0.01)
    now = seen["joint_states"]["msg"].position
    peak = max(peak, now[JOINT] - start[JOINT])
    if max(abs(now[i] - start[i]) for i in range(6) if i != JOINT) > 0.03:
        print("another joint moved; stopping")
        break
pub.publish(Float64MultiArray(data=start))       # leave the arm holding where it began
spin(1.0)
end = seen["joint_states"]["msg"].position
print("joint %d: commanded %+.3f rad, peak measured %+.4f rad, final offset %+.4f rad"
      % (JOINT, AMPLITUDE, peak, end[JOINT] - start[JOINT]))
