# Policy server — integration contract

> This is what CloudChef needs from a partner-trained policy so we can run it on our
> robots and through our eval loop. It is self-contained: no
> CloudChef codebase knowledge assumed. Items marked **⚠️ partner input** are open
> questions for you. A standalone smoke client ([`policy_smoke.py`](policy_smoke.py), ~40 lines,
> zmq + msgpack + opencv only) lets you test your server without any CloudChef code.

## Two ways to integrate

| | What you build | Needs a robot to develop | Document |
|---|---|---|---|
| **Policy server** (recommended) | An inference server that speaks our wire: observations in, joint targets out, 30 Hz | No | this page |
| **Arm driver** | Your own ROS 2 node that reads the robot's topics and commands the arm directly, up to 200 Hz | Yes — rig access, which needs approval from the CloudChef team | [ARM_DRIVER_INTERFACE.md](ARM_DRIVER_INTERFACE.md) |

Start with the policy server unless you need command rates above 30 Hz, torque
feedback at source rate, or control over arm stiffness. The rest of this page is
the policy-server contract.

## The ask in one paragraph

Package your policy as a **long-lived inference server process** that speaks our
existing ZMQ + msgpack wire (described below). Our robot already has a client for
this wire, so on our side the integration is one ~100-line launcher and a config
file — no new client code, no changes to your model code after handoff. You own
everything model-side (preprocessing, action chunking, normalisation); we own
everything robot-side (cameras, joint I/O, safety, lifecycle, eval).

## Topology

```
  robot computer (no GPU)                     CloudChef GPU box (RTX 5090, 32 GB)
 ┌───────────────────────────┐                ┌────────────────────────────────────┐
 │ control loop @ 30 Hz      │   LAN, ZMQ     │ your server, your venv, your repo  │
 │   our client ─────────────┼───────────────►│   python -m <pkg>.deploy.server    │
 │   (one request per tick)  │  REQ ◄──► REP  │   --server-args-json <file>        │
 └───────────────────────────┘                └────────────────────────────────────┘
```

**Today we start your server by hand** on the GPU box (the command above, in your
venv) and our client attaches to it once the ready line has appeared. Automated
launch — our launcher spawning your server as a subprocess, tailing stdout for the
ready line, and reusing a warm server through the handshake (§3.3) — is the intended
end state, and the process contract in §2 is written for it; build to §2 so nothing
changes on your side when we switch. The client sends one observation per control
tick and expects one action back. Sometimes both halves run on the same box; the wire
is identical either way.

## 1. What you deliver

1. **A git repo (or tarball)** containing a module entrypoint
   `python -m <pkg>.deploy.server --server-args-json <path>` plus everything it imports.
2. **Its own Python environment**, reproducible: `uv.lock` + `pyproject.toml` preferred
   (`uv sync` must produce a working `.venv/bin/python`), or a pinned
   `requirements.txt` with the Python version stated. Target: Linux x86_64, CUDA 12.x,
   RTX 5090 (sm_120 — check your torch build supports it). Nothing from your env is
   installed into ours.
3. **Checkpoint bundle**: a directory with the weights and any stats/config files the
   server needs; the server takes its path from `server_args` (below). No downloads
   at runtime — the GPU box may be offline from the public internet.
4. **A short README**: the `server_args` keys you accept (with defaults), which cameras
   the model uses, the action layout, expected VRAM, and expected per-call latency.

## 2. Process contract

| Item | Requirement |
|---|---|
| Config | **One JSON file**, path given by `--server-args-json`. Every runtime knob (checkpoint path, port, device, chunk length, …) is a key in it. No other CLI flags, no env vars required. Our config system writes this file verbatim from a profile. |
| Required keys | `port` (int). Everything else is yours to define; we will pass through whatever your README lists. |
| Bind | `tcp://0.0.0.0:<port>` — the robot reaches the GPU box over the LAN. |
| Ready line | Print **exactly** `[PolicyServer] Listening on tcp://0.0.0.0:<port>` to stdout **after** the socket is bound and the model is loaded and warmed (first inference done). Nothing is sent before it appears (an operator watches for it today; the launcher will tail stdout for it). Keep it on one line, under 80 columns, with stdout unbuffered (`PYTHONUNBUFFERED=1` is set for you; flush anyway). |
| Startup budget | Under 60 s load-to-ready is what we see from comparable models. The launcher's default wait is **180 s**; it is configurable per model up to about 10 minutes — tell us your number if you need more than 180 s. |
| Logging | stdout/stderr only (we capture both to a file). Log per-call latency at least every ~30 calls. |
| Lifecycle | Serve forever until killed. You are stopped with SIGKILL (the launcher uses `fuser -k` on the port), so never rely on shutdown hooks. Crashing is fine; hanging is not — a request that cannot be served must still get a reply (an error reply is OK; see §3.4). |
| Concurrency | **One client, one request in flight, one episode at a time.** A plain single-threaded REQ/REP loop is exactly right. |

## 3. Wire protocol

ZMQ **REP** socket on your side (our client is REQ). Every message is one msgpack map
(`use_bin_type=True`). Arrays travel as **raw little-endian bytes** — there are no
nested ndarray objects; the receiver knows the dtype from this document.

Three request kinds, distinguished by the `type` key (absent for observations).

### 3.1 Observation → action (every control tick, ~30 Hz)

Request (no `type` key):

```
{
  "qpos":           <bytes>  float32 LE, length qpos_dim   — current joint positions
  "qvel":           <bytes>  float32 LE, length qpos_dim   — always zeros; ignore
  "qpos_dim":       <int>    7 or 14 (see §4)
  "images":         { "<camera>": <bytes JPEG> , ... }     — standard JPEG, quality 90 (cv2.imdecode gives BGR; convert to RGB)
  "image_encoding": "jpeg"
  "arm_mode":       "right" | "left" | "both"              — see §4
  "timestamp_ns":   <int>    robot wall clock (Unix epoch ns); 0 in warmup observations
  "efforts":        <bytes>  float32 LE, optional           — joint torques; ignore unless you train on them
}
```

Camera keys are our canonical names and arrive as-is: `overhead`, `wrist_left`,
`wrist_right`, `front`. **Read the ones your model uses and ignore the rest** — the
set of cameras sent is a robot-side setting and may include extras. **Frame size
depends on the robot and on the stream we configure for your model**, so do not
assume one: RealSense robots send 480×848 or 480×640; ZED robots are 1920×1200 at
the sensor and we normally send a downscaled stream (448×448 letterboxed, 640×384
or 480×480 today). Tell us your training input geometry (§9) and we will pick the
closest stream; **resize, crop, and normalise server-side**, exactly as training
did. The JPEG you receive has been decoded and re-encoded at quality 90 on the
robot.

Reply:

```
{
  "action":       <bytes>  float32 LE, length qpos_dim   — absolute joint targets for THIS tick
  "timestamp_ns": <int>    echo of the request's
  "actions":      <bytes>  float32 LE, T × qpos_dim row-major, OPTIONAL   — lookahead (§3.1.1)
  "extras":       { "<key>": <bytes float32 LE>, ... }, OPTIONAL          — telemetry (§3.1.2)
}
```

**One action per call.** If your policy predicts a chunk of N future actions, keep the
chunk server-side, pop one per request, and re-run the model when the chunk drains (or
on whatever replan schedule you trained for). The client does not know or care about
your chunking.

#### 3.1.1 `actions` lookahead (optional, recommended)

The remaining queued actions as of this tick, **inclusive of `action`** — row 0 must be
bit-identical to `action`; the client rejects the reply otherwise. Within a chunk `T`
shrinks by one per tick; a jump back up means you replanned. We use it for
smoothing and replan detection. Omit the key entirely if you have no queue.

#### 3.1.2 `extras` (optional)

String keys to float32 arrays only (any length, including 1-element scalars). Keys
starting with `latency_` are lifted into our telemetry automatically; e.g.
`latency_infer_ms` (model forward time) and `latency_total_ms` (request to reply) are
worth sending. Anything else is recorded alongside each episode for your debugging,
**except these reserved keys, which our client acts on — do not use them for
anything else:**

| Key | What our client does with it |
|---|---|
| `reward` | Treated as a success probability. By default, a value ≥ 0.9 on 3 consecutive ticks **ends the episode**: the client holds position and stops calling you until the next reset. Send it only if you mean that; tell us if you want the stop disabled or the threshold changed. |
| `reward_binary` | Written by our client; do not send. |
| `ee_pose_raw` | Interpreted as an end-effector pose for telemetry. |
| any key starting with `_meta__` | Reserved for our own metadata. |

### 3.2 Reset (once per episode, before the first observation)

```
request:  {"type": "reset", "timestamp_ns": 0}
reply:    {"type": "reset_ack", "timestamp_ns": 0}
```

Clear **all** episode state: action queue, observation/frame history, step counters,
temporal ensembles. Guarantee: the next observation is step 0 of a fresh episode.

### 3.3 Handshake (optional; enables warm reuse)

Our launcher, on finding a server already on the port, asks whether it is the right one:

```
request:  {"type": "handshake", "server_args": { ...the JSON we would launch you with... }}
reply:    {"type": "handshake_ack", "match": true | false}
```

Compare the incoming `server_args` to the ones you were started with (keys that change
what you compute — checkpoint, chunk length, …; ignore `port`). `match: true` → we
reuse you and skip the model reload (this is what makes back-to-back eval episodes
start in seconds). `match: false` → reply, then **exit** so we can respawn with the
new args. If you skip this message type, reply with any map; we will kill and respawn
every time — correct but slower.

### 3.4 Errors

Never leave a request unanswered (a REQ/REP client is stuck until it gets a reply).
For a request you cannot serve, reply
`{"error": "<message>"}` — the client treats a reply without `action` as a failed tick
and **holds the robot's current position** for that tick. Log the cause.

## 4. Joint layout and `arm_mode`

Our robots are dual-arm with 7 values per arm: 6 arm joints then the gripper.
`qpos` is the same layout and units as the recorded episodes you already have from us
(absolute joint positions in radians, gripper normalised). Actions are absolute joint
targets in the same layout.

`arm_mode` is a profile setting on our side that selects what you see:

| `arm_mode` | `qpos_dim` | `qpos` you receive | `action` you return |
|---|---|---|---|
| `"right"` | 7 | right arm `[j0..j5, gripper]` | 7 values, right arm |
| `"left"` | 7 | left arm | 7 values, left arm |
| `"both"` | 14 | `[left_7, right_7]` | 14 values |

The idle arm is held by the client; you never see it. **⚠️ partner input:** tell us
which mode your model is trained for; the recorded episodes you received from us are right-arm
active with a 14-D recording layout, so `"right"` (7-D) or `"both"` (14-D) both work —
pick the one matching your training target.

If your model does not predict the gripper, return the input gripper value unchanged;
we can pin the gripper robot-side.

## 5. Timing

- The client calls you at **30 Hz** (a per-model setting; tell us if you trained at
  another rate) and waits up to a configurable timeout, **5 s by default** (we set
  it to cover your worst case — tell us your p99). Steady-state replies under ~25 ms
  keep the loop real-time; a chunk-popping tick should be ~1 ms. Treat a missed
  timeout as fatal for the episode: always reply, even with an error (§3.4).
- The **first observation after a reset** may be slow (full forward pass, maybe a JIT
  shape miss); tell us the number and keep it under ~5 s.
- On connect our client sends a **warmup**: two observations, then a reset. Each has
  all-zero `qpos`, all-zero `efforts`, `timestamp_ns` 0 and black JPEG frames for
  every camera configured for your model. **By default the warmup `qpos` is 7-D and
  the frames are 480×640, whatever your `arm_mode` and real frame size** — we
  override both per model, so tell us the `qpos_dim` and frame size your server
  needs to see (§9). The warmup must not crash the server, and an error reply to it
  fails the connect; the actions it produces are discarded.

## 6. What we handle (so you do not have to)

Camera naming, idle-arm padding, holding position on any error or timeout, the
gripper pin, process spawn/reuse/kill, LAN addressing, telemetry recording, and the
eval harness that replays recorded episodes against your server for parity checks.

## 7. Scope guard — things that would change the shape

The contract above assumes **RGB in, joint targets out, state held server-side**. If
any of these apply, tell us **before** building, because each one moves work onto our
side (a custom client, ~150+ lines, plus a rig session):

- the model needs **depth** or any non-RGB sensor;
- the model acts in **end-effector / Cartesian space** rather than joints;
- the model needs **client-side frame history** (stacked past frames at a fixed stride);
- you want to **host the model yourselves** and expose an HTTP/WebSocket endpoint
  instead of shipping a repo.

## 8. Acceptance checklist (run before handoff)

1. `uv sync` (or your documented equivalent) on a clean Linux box produces a working venv.
2. `python -m <pkg>.deploy.server --server-args-json args.json` prints the ready line.
3. `policy_smoke.py` runs reset → 60 observations → reset against it with no
   errors and prints per-call latency.
4. Replaying a recorded episode's frames + `qpos` through the server reproduces your
   offline predictions (this is the parity test we will repeat on our side).
5. VRAM at steady state fits in **16 GB** (the 5090 is shared). **⚠️ partner input:**
   actual number.

## 9. Open questions for you

1. `arm_mode` / action layout (§4).
2. Which cameras the model consumes, and the training input resolution per camera.
3. Chunk length and replan schedule (every N ticks? on drain?). Will you send `actions`?
4. Steady-state and first-call latency on a 5090; VRAM; load-to-ready time.
5. The `qpos_dim` and frame size your server needs in the warmup (§5), and whether
   you send a `reward` extra (§3.1.2).
6. Anything in §7.

## Smoke client

[`policy_smoke.py`](policy_smoke.py) sends the same message shapes as our robot: reset → 60
observations → reset. It is a shape check, not a full stand-in — it sends no warmup
(§5), no handshake (§3.3) and no `efforts`, always sends four cameras at 480×848,
and waits 60 s per call where the robot waits 5 s. Deps: `pip install pyzmq msgpack numpy opencv-python`.

```bash
python policy_smoke.py tcp://localhost:<port>        # edit arm_mode / dim at the top for 14-D
```
