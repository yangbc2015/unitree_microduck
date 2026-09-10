#!/usr/bin/env python3
"""Hand-rolled JSON-RPC 2.0 client to exercise robotd's IPC layer on Jetson.
Sends hello → robot.health → robot.modelApi → robot.subscribe, reads responses.
Verifies the non-hardware part of the microduck software stack is alive.
"""
import json, socket, sys, time

SOCK = sys.argv[1] if len(sys.argv) > 1 else "/tmp/robotd-test.sock"
API_VERSION = 16  # from duck-ipc-proto API_VERSION const

def recv_line(sock):
    buf = b""
    while not buf.endswith(b"\n"):
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    return buf.decode("utf-8", "replace").strip()

def rpc_call(sock, req_id, method, params, timeout=3):
    req = {"jsonrpc": "2.0", "id": req_id, "method": method, "params": params}
    sock.sendall((json.dumps(req) + "\n").encode())
    sock.settimeout(timeout)
    lines = []
    while True:
        line = recv_line(sock)
        if not line:
            break
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        lines.append(obj)
        # a response carries "id" matching ours
        if obj.get("id") == req_id and "result" in obj:
            break
        if obj.get("id") == req_id and "error" in obj:
            break
    return lines

results = {}

# --- hello handshake (first call on every connection) ---
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(SOCK)
resp = rpc_call(s, 1, "hello", {"api_version": API_VERSION})
results["hello"] = resp
print("hello ->", json.dumps(resp, ensure_ascii=False)[:300])

# --- robot.health : request/response ---
resp = rpc_call(s, 2, "robot.health", {})
results["robot.health"] = resp
print("robot.health ->", json.dumps(resp, ensure_ascii=False)[:500])

# --- robot.modelApi : request/response ---
resp = rpc_call(s, 3, "robot.modelApi", {})
results["robot.modelApi"] = resp
print("robot.modelApi ->", json.dumps(resp, ensure_ascii=False)[:300])

# --- robot.subscribe : stream (notifications carry no id) ---
req = {"jsonrpc": "2.0", "id": 4, "method": "robot.subscribe", "params": {"hz": 10}}
s.sendall((json.dumps(req) + "\n").encode())
s.settimeout(2.5)
frames = 0
notif_lines = []
start = time.time()
try:
    while time.time() - start < 2.5:
        line = recv_line(s)
        if not line:
            break
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if obj.get("method") == "robot.state":
            frames += 1
            notif_lines.append(line)
except socket.timeout:
    pass
print(f"robot.subscribe -> got {frames} robot.state notification frames in 2.5s")
if notif_lines:
    print("first frame ->", notif_lines[0][:400])
s.close()

# verdict
ok = True
for name in ("hello", "robot.health", "robot.modelApi"):
    items = results.get(name) or []
    last = items[-1] if items else {}
    status = "OK" if ("result" in last and "error" not in last) else "FAIL"
    if status == "FAIL":
        ok = False
    print(f"  [{status}] {name}")
print("  [{}] robot.subscribe ({} frames)".format("OK" if frames > 0 else "EMPTY", frames))
sys.exit(0 if ok else 1)
