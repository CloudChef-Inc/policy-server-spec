# Arm driver — direct ROS 2 integration

> This is the deeper of the two ways to run your software on a CloudChef robot.
> Instead of serving a policy behind our wire ([README](README.md)), you run your
> own ROS 2 node on the robot's network: you read the arm and camera topics at
> their native rate and command the arm driver yourself.
>
> **This level needs rig access, and rig access needs approval from the CloudChef
> team.** Nothing here can be exercised without a robot; ask us before you build
> against it. Site details (addresses, ROS domain, robot model, per-robot limits)
> are handed over when access is approved, not in this document.

Written against CloudChef's robot stack at commit `a2f276923` (2026-11-25). Topic
names and limits can change; we will tell you when they do.

## When to pick this over the policy server

| | Policy server ([README](README.md)) | Arm driver (this document) |
|---|---|---|
| Your code runs | on a GPU box, behind a socket | as a ROS 2 node on the robot's network |
| Observations | joint positions + JPEG frames, one set per 30 Hz tick | every topic at its own rate (joint state at 200 Hz) |
| Commands | absolute joint targets, 30 Hz | joint targets, impedance references or trajectories, up to 200 Hz |
| Smoothing and velocity limiting | ours | **yours** |
| Hold on error or timeout | ours | **yours** (see [Safety](#safety-read-this-first)) |
| Needs a robot to develop | no — `policy_smoke.py` stands in | yes |

Pick the policy server unless you need something it cannot give you: command
rates above 30 Hz, torque or contact feedback at source rate, control over
stiffness, or your own trajectory execution.

## Safety: read this first

At this level your node is the only thing between your software and the motors.

- **The arm holds the last target it received, indefinitely.** There is no
  stale-command timeout. If your node stops publishing, crashes or loses the
  network, the arm keeps holding the last commanded position.
- **Nothing smooths your commands.** A target far from the current position is
  executed as a step, at whatever speed the motors can reach. Rate-limit and
  interpolate on your side.
- **One writer per arm.** We make sure none of our own nodes command an arm
  while you have it. Do not run two of your own writers against one arm.
- **A CloudChef operator is present for every session**, with the emergency stop.
  We set the control mode, stiffness and limits for the session before you
  connect; tell us in advance what you need.
- **Start every session by reading, not writing.** Subscribe, confirm joint
  order and signs against the physical arm, and make your first command equal to
  the measured position.

## Setup

| Item | Value |
|---|---|
| ROS 2 distribution | Jazzy |
| Middleware | `rmw_cyclonedds_cpp` |
| ROS domain ID, network addresses | given at approval |
| Arm namespaces | `/components/follower_left`, `/components/follower_right` |
| Custom message types | the [`ccros_msgs`](ccros_msgs/) package in this repo — build it in your own workspace |

Only the impedance reference, the gains service, the control-mode service and
the gains-carrying trajectory action use `ccros_msgs`. Everything else is
`std_msgs`, `sensor_msgs` and `control_msgs`. The package here is a subset of
ours with identical definitions: build it on your machine, never into a
workspace on our robot, where it would shadow the full package.

## Joint layout

Each arm has 6 revolute joints and a gripper.

| Index | Name on `joint_states` | Unit |
|---|---|---|
| 0–5 | `<arm>/joint_0` … `<arm>/joint_5` | rad |
| 6 | `<arm>/left_carriage_joint` (the gripper) | normalised, 0 = closed, 1 = open |

`<arm>` is `follower_left` or `follower_right`. This is the same layout as the
recorded episodes and as `qpos` in the policy-server wire.

## What you read

All topics below are relative to the arm namespace.

| Topic | Type | Rate | Content |
|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | 200 Hz | 7 names as above; `position`, `velocity` and `effort` (motor torque, N·m) all filled |
| `external_efforts` | `std_msgs/Float64MultiArray` | 200 Hz | the same 7 effort values |
| `joint_temperatures` | `std_msgs/Float64MultiArray` | ~2 Hz | per-joint motor temperature, °C |
| `reference_controller/limits` | `std_msgs/Float64MultiArray`, latched | on change | 12 values: lower limits of joints 0–5, then upper limits, rad. Impedance-enabled arms only |

Cameras publish under `/components/cam_overhead`, `/components/cam_wrist_left`
and `/components/cam_wrist_right`:

| Topic | Type |
|---|---|
| `color/image_rect_raw/compressed` (overhead), `color/image_raw/compressed` (wrists) | `sensor_msgs/CompressedImage` (JPEG) |
| `color/camera_info` | `sensor_msgs/CameraInfo` |
| `aligned_depth_to_color/image_raw` (overhead only) | `sensor_msgs/Image`, depth registered to colour |

Topic names and resolutions can differ per robot, and downscaled colour streams
are available; we give you the list for the rig you are approved on. Use each
message's header stamp, not your receive time, to align streams.

## What you write

Pick **one** of the three command paths per arm. All command subscriptions are
reliable with a queue depth of 1: only the newest message matters, and the
driver consumes at most one command per 5 ms.

What happens to a bad command differs by path:

| Path | Target outside the joint limits | Non-finite value |
|---|---|---|
| Position targets | clipped to the limit | message refused |
| Gripper | clamped to [0, 1] | message ignored |
| Impedance reference | `position` outside the limits: message **refused**. `command` joints: clipped | message refused |
| Trajectory | not checked at accept; the motor command is clipped during execution, which can abort the goal on the path tolerance | — |

Refused means the previous target stays in force. Do not use clipping as a
limiter: stay inside the limits yourself.

### 1. Position targets

| Topic | Type | Content |
|---|---|---|
| `forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 6 joint targets, rad, order `joint_0..joint_5` |
| `gripper_forward_controller/commands` | `std_msgs/Float64MultiArray` | 1 value, normalised 0..1 |

The simplest path. Each message replaces the arm's target; the driver's position
controller tracks it at the arm's configured gains. Publish at a steady rate
(100–200 Hz recommended) and keep consecutive targets close together.

### 2. Impedance references

| Topic | Type |
|---|---|
| `reference_controller/commands` | [`ccros_msgs/JointReference`](ccros_msgs/msg/JointReference.msg) |

Available only on arms we have started with impedance control enabled — say so
when you request access. One message carries the position target (`command`,
6 joints + gripper) together with a reference position, velocity and
acceleration for the 6 arm joints.

What you can rely on:

- The arm follows the reference with a spring whose pull is capped: when the arm
  is blocked, the force it applies stops growing once the reference is more than
  a fixed angle ahead of the measured position. This is what makes contact with
  a surface soft.
- Overall stiffness is one scale factor, set by us per session.
- A reference is extrapolated with its velocity and acceleration for at most
  10 ms, then held at rest. Publish faster than that for continuous motion.
- A message with a non-finite value, a reference `position` outside the joint
  limits, or a gripper value outside [0, 1] is **refused**: the previous target
  stays in force. Refusals are counted and reported to our operator, not to
  you — check your values against `reference_controller/limits` before sending.
- If the arm is held away from its reference for about half a second, the
  driver can latch a hold at the measured position instead of continuing to
  push. Tell the operator if your task presses on a surface for longer.

### 3. Trajectories

| Action | Type |
|---|---|
| `follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `follow_joint_trajectory_gains` | [`ccros_msgs/action/FollowJointTrajectoryGains`](ccros_msgs/action/FollowJointTrajectoryGains.action) |

The driver splines the trajectory and executes it at control rate. The second
action carries position gains in the goal — but on the rigs this document
covers, goal gains are not applied: send **empty** `gains` (a goal with
non-empty gains is rejected). Trajectories run at the arm's configured gains.

A goal is rejected when:

- the arm is not in `position` control mode;
- it has fewer than 2 points;
- `joint_names` is not exactly the 6 arm joints (bare `joint_0` or prefixed
  `follower_right/joint_0`, any order);
- `time_from_start` is not strictly increasing;
- another trajectory is active (cancel it first — one goal at a time);
- gains set with `set_arm_gains` are in force on the arm;
- the goal carries non-empty gains (see above).

An accepted goal is aborted if the arm leaves the path by more than 0.25 rad on
any joint, or does not settle within 0.05 rad of the final point within 0.5 s
of the trajectory's end.

Trajectories move the 6 arm joints only; command the gripper on its topic.

## Stiffness on the position path

| Service | Type |
|---|---|
| `set_arm_gains` | [`ccros_msgs/srv/SetArmGains`](ccros_msgs/srv/SetArmGains.srv) |

Sets the proportional and derivative gains of the 6 arm joints. Stock gains are
the maximum; each joint can be lowered to one eighth of stock:

| Joint | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| kp stock, N·m/rad | 500 | 500 | 500 | 200 | 80 | 40 |
| kp minimum | 62.5 | 62.5 | 62.5 | 25 | 10 | 5 |
| kd stock, N·m·s/rad | 5 | 5 | 5 | 3 | 1.0 | 1.5 |
| kd minimum | 1.76 | 1.76 | 1.76 | 1.06 | 0.35 | 0.53 |

Values outside the range are clamped, and the response reports what was applied
— trust `applied_kp` / `applied_kd` over this table.

- The call is refused unless the arm is in `position` mode with no trajectory
  active. When it succeeds the arm is commanded to hold its measured position.
- Fill `owner` with your node's fully-qualified name: the driver restores the
  configured gains by itself one to two seconds after that node leaves the ROS
  graph. A node that crashes without shutting down is only seen as gone once
  the middleware times it out, which takes longer.
- Any `set_control_mode` call also ends the gains.
- While gains set this way are in force, trajectory goals are rejected.

Lower gains mean larger tracking error under load; the arm will sag further
from its target.

## Control mode

| Service | Type |
|---|---|
| `set_control_mode` | [`ccros_msgs/srv/SetControlMode`](ccros_msgs/srv/SetControlMode.srv) |

`"position"` or `"zero_torque"` (position gains go to zero; the arm is held up
by gravity compensation only and can be moved by hand). Our operator sets the
mode at the start of a session. Do not switch it without telling the operator:
in `zero_torque` the arm no longer holds its position, and anything it carries
that the gravity model does not know about will pull it down.

## What we give you at approval

- network access to one rig, its addresses and ROS domain ID;
- the robot model (URDF) and end-effector frame for that rig;
- its joint limits, the camera topic list and resolutions;
- which command paths are enabled on it, and the stiffness we will run;
- a session slot with an operator.

## Questions we will ask you

1. Which command path, and at what rate?
2. Do you need impedance control, and how stiff?
3. Which cameras, and do you need depth?
4. Where does your node run — your machine on the robot's network, or ours?
5. What does your node do when it loses the robot's state for more than a few
   control periods?
