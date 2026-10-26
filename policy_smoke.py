"""Standalone smoke client for the CloudChef policy-server contract (see README.md).

Usage: python policy_smoke.py tcp://<host>:<port>
"""
import sys, time, msgpack, numpy as np, zmq, cv2

url = sys.argv[1] if len(sys.argv) > 1 else "tcp://localhost:5562"
arm_mode, dim = "right", 7                      # or "both", 14
cams = ("overhead", "wrist_left", "wrist_right", "front")

def jpeg(h=480, w=848):
    frame = np.random.randint(0, 255, (h, w, 3), np.uint8)    # BGR for imencode
    return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()

ctx = zmq.Context(); s = ctx.socket(zmq.REQ)
s.setsockopt(zmq.RCVTIMEO, 60_000); s.setsockopt(zmq.LINGER, 0); s.connect(url)

def rpc(msg):
    s.send(msgpack.packb(msg, use_bin_type=True)); return msgpack.unpackb(s.recv(), raw=False)

assert rpc({"type": "reset", "timestamp_ns": 0}).get("type") == "reset_ack"
qpos = np.zeros(dim, np.float32)
for i in range(60):
    t0 = time.perf_counter()
    r = rpc({"qpos": qpos.tobytes(), "qvel": np.zeros(dim, np.float32).tobytes(),
             "qpos_dim": dim, "images": {c: jpeg() for c in cams},
             "image_encoding": "jpeg", "arm_mode": arm_mode, "timestamp_ns": time.time_ns()})
    ms = (time.perf_counter() - t0) * 1e3
    assert "action" in r, r
    a = np.frombuffer(r["action"], "<f4"); assert a.shape == (dim,), a.shape
    if "actions" in r:
        la = np.frombuffer(r["actions"], "<f4").reshape(-1, dim)
        assert np.array_equal(la[0], a), "actions[0] must equal action"
    print(f"tick {i:02d}  {ms:7.1f} ms  action[:3]={a[:3]}  lookahead={len(la) if 'actions' in r else '-'}")
    qpos = a                                      # open-loop: feed the prediction back
assert rpc({"type": "reset", "timestamp_ns": 0}).get("type") == "reset_ack"
print("OK")
