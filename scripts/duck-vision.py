#!/usr/bin/env python3
"""Receive camera frames from `mediad --stream-to` and analyze them with a VLM.

This is the local-model half of microduck's camera stream: mediad dials out with
JPEG frames over a WebSocket, this script answers that WebSocket, and every
`--interval` seconds it sends the newest frame to an OpenAI-compatible
multimodal endpoint (default: a llama.cpp server on this board). Stdlib only —
no websockets/Pillow/requests dependency — so it runs on a stock Jetson python3.

Wire protocol (mediad/src/stream.rs):
  1. client connects to ws://HOST:PORT/PATH
  2. client sends one text frame: JSON hello (robot + frames metadata)
  3. client sends binary frames: one complete JPEG per message
  4. this script may send text frames back; mediad counts/logs them and ignores
"""

import argparse
import base64
import hashlib
import json
import os
import signal
import socket
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
DEFAULT_PROMPT = (
    "你是机器鸭上的本地视觉助手。用中文简洁描述这张摄像头画面：场景、明显物体、"
    "有无线缆/障碍/人；如果画面偏暗、偏紫或看不清，直接说明。"
)
DEFAULT_MODEL_URL = os.environ.get("LLAMA_URL", "http://127.0.0.1:8081/v1")


def log(*parts):
    print(time.strftime("[%H:%M:%S]"), *parts, flush=True)


# ── minimal WebSocket server (RFC 6455), enough for one mediad client ─────────


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf


def read_http_request(conn):
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(4096)
        if not chunk:
            raise ConnectionError("closed during handshake")
        data += chunk
        if len(data) > 65536:
            raise ConnectionError("handshake too large")
    head = data.split(b"\r\n\r\n", 1)[0].decode("iso-8859-1", "replace")
    lines = head.split("\r\n")
    method, path, _version = (lines[0].split(" ", 2) + ["", ""])[:3]
    headers = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    return method, path, headers


def accept_key(sec_key):
    digest = hashlib.sha1((sec_key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def send_frame(conn, lock, opcode, payload=b""):
    first = 0x80 | (opcode & 0x0F)  # FIN + opcode; server frames are never masked
    length = len(payload)
    if length < 126:
        header = struct.pack("!BB", first, length)
    elif length <= 0xFFFF:
        header = struct.pack("!BBH", first, 126, length)
    else:
        header = struct.pack("!BBQ", first, 127, length)
    with lock:
        conn.sendall(header + payload)


def send_text(conn, lock, obj):
    send_frame(conn, lock, 0x1, json.dumps(obj, ensure_ascii=False).encode("utf-8"))


def read_frame(conn):
    b1, b2 = recv_exact(conn, 2)
    fin = bool(b1 & 0x80)
    opcode = b1 & 0x0F
    masked = bool(b2 & 0x80)
    length = b2 & 0x7F
    if length == 126:
        (length,) = struct.unpack("!H", recv_exact(conn, 2))
    elif length == 127:
        (length,) = struct.unpack("!Q", recv_exact(conn, 8))
    mask = recv_exact(conn, 4) if masked else b""
    payload = recv_exact(conn, length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
    return fin, opcode, payload


def read_message(conn, send_lock):
    """Return (opcode, payload) for a complete text/binary message.

    Handles ping/pong and fragmentation internally. Raises ConnectionError on close.
    """
    frag_opcode = None
    frag = bytearray()
    while True:
        fin, opcode, payload = read_frame(conn)
        if opcode == 0x9:  # ping
            send_frame(conn, send_lock, 0xA, payload)
            continue
        if opcode == 0xA:  # pong
            continue
        if opcode == 0x8:  # close
            try:
                send_frame(conn, send_lock, 0x8, payload[:125])
            finally:
                raise ConnectionError("peer sent close")
        if opcode in (0x1, 0x2):
            if fin:
                return opcode, payload
            frag_opcode = opcode
            frag = bytearray(payload)
            continue
        if opcode == 0x0 and frag_opcode is not None:
            frag.extend(payload)
            if fin:
                return frag_opcode, bytes(frag)
            continue
        # Unknown control/frame: ignore rather than kill the stream.


def is_jpeg(data):
    return len(data) >= 4 and data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"


# ── OpenAI-compatible multimodal endpoint ─────────────────────────────────────


class VisionClient:
    def __init__(self, base_url, model, prompt, max_tokens, timeout):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.prompt = prompt
        self.max_tokens = max_tokens
        self.timeout = timeout

    def resolve_model(self):
        if self.model:
            return self.model
        try:
            with urllib.request.urlopen(self.base_url + "/models", timeout=5) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            items = data.get("data") or []
            if items and items[0].get("id"):
                self.model = items[0]["id"]
        except Exception as why:  # best-effort: llama.cpp ignores the model field anyway
            log("model list failed, falling back to 'local':", why)
            self.model = "local"
        return self.model

    def analyze(self, jpeg):
        payload = {
            "model": self.resolve_model(),
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii")
                            },
                        },
                    ],
                }
            ],
            "max_tokens": self.max_tokens,
            "temperature": 0.2,
        }
        request = urllib.request.Request(
            self.base_url + "/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as why:
            body = why.read().decode("utf-8", "replace")[:400]
            raise RuntimeError(f"vision endpoint HTTP {why.code}: {body}") from why
        except Exception as why:
            raise RuntimeError(f"vision endpoint unreachable: {why}") from why
        message = (data.get("choices") or [{}])[0].get("message") or {}
        return message.get("content") or message.get("reasoning_content") or ""


# ── connection handling ───────────────────────────────────────────────────────


class Server:
    def __init__(self, args):
        self.args = args
        self.stop = threading.Event()
        self.client = VisionClient(
            args.model_url, args.model, args.prompt, args.max_tokens, args.timeout
        )

    def save_frame(self, jpeg, seq):
        if not self.args.save_dir:
            return None
        name = time.strftime("duck-%Y%m%d-%H%M%S") + f"-{seq:06d}.jpg"
        path = os.path.join(self.args.save_dir, name)
        with open(path, "wb") as out:
            out.write(jpeg)
        return path

    def handle_hello(self, payload):
        try:
            hello = json.loads(payload.decode("utf-8", "replace"))
        except json.JSONDecodeError:
            log("hello (not JSON):", payload[:120])
            return
        robot = hello.get("robot") or {}
        frames = hello.get("frames") or {}
        log(
            "hello:",
            robot.get("name") or robot.get("serial") or "microduck",
            "frames=",
            frames.get("encoding"),
            f"fps={frames.get('fps')}",
            f"longest={frames.get('longest')}",
            f"mount_rotate={frames.get('mount_rotate')}",
        )

    def analyze_and_report(self, conn, send_lock, jpeg, seq):
        saved = self.save_frame(jpeg, seq)
        started = time.time()
        try:
            text = self.client.analyze(jpeg).strip()
        except Exception as why:
            log(f"analysis failed for frame {seq}:", why)
            return False
        elapsed = time.time() - started
        log(f"frame {seq} ({len(jpeg)} bytes, {elapsed:.1f}s)" + (f", saved {saved}" if saved else ""))
        print(text, flush=True)
        try:
            send_text(
                conn,
                send_lock,
                {"type": "vision", "seq": seq, "elapsed_s": round(elapsed, 3), "analysis": text},
            )
        except Exception as why:
            log("could not send analysis back to mediad:", why)
            return False
        return True

    def handle_client_once(self, conn):
        send_lock = threading.Lock()
        seq = 0
        conn.settimeout(max(self.args.wait, 10.0))
        try:
            while True:
                opcode, payload = read_message(conn, send_lock)
                if opcode == 0x1:
                    self.handle_hello(payload)
                    continue
                if opcode != 0x2:
                    continue
                seq += 1
                if len(payload) < self.args.min_bytes or not is_jpeg(payload):
                    log(f"frame {seq}: not a usable JPEG ({len(payload)} bytes), waiting for the next")
                    continue
                ok = self.analyze_and_report(conn, send_lock, payload, seq)
                try:
                    send_frame(conn, send_lock, 0x8, b"done")
                except Exception:
                    pass
                return ok
        except (ConnectionError, socket.timeout) as why:
            log("once mode ended before a frame:", why)
            return False
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def handle_client(self, conn, addr):
        send_lock = threading.Lock()
        state = {"latest": None, "seq": 0, "analyzed": 0, "closed": False}
        state_lock = threading.Lock()
        new_frame = threading.Event()

        def analyzer():
            while not self.stop.is_set():
                new_frame.wait(max(self.args.interval, 0.2))
                new_frame.clear()
                with state_lock:
                    if state["closed"] and state["seq"] == state["analyzed"]:
                        return
                    if state["latest"] is None or state["seq"] == state["analyzed"]:
                        continue
                    jpeg = state["latest"]
                    seq = state["seq"]
                    state["analyzed"] = seq
                started = time.monotonic()
                self.analyze_and_report(conn, send_lock, jpeg, seq)
                with state_lock:
                    if state["closed"]:
                        return
                # `--interval` is a minimum spacing between analyses, not just a wait for a frame.
                # Frames arriving at 1 fps make the wait return immediately, so without this the loop
                # runs at the model's own speed - and on a board where the model's memory is shared
                # with everything else, the analyser's request rate is the lever that matters (a
                # local llama.cpp keeps roughly 50 MiB per multimodal request and never gives it
                # back). Frames that arrive inside the interval are dropped; newest still wins.
                remaining = self.args.interval - (time.monotonic() - started)
                if remaining > 0 and self.stop.wait(remaining):
                    return

        worker = threading.Thread(target=analyzer, name="duck-vision-analyze", daemon=True)
        worker.start()
        try:
            while not self.stop.is_set():
                opcode, payload = read_message(conn, send_lock)
                if opcode == 0x1:
                    self.handle_hello(payload)
                    continue
                if opcode != 0x2:
                    continue
                if len(payload) < self.args.min_bytes or not is_jpeg(payload):
                    log(f"bad frame ({len(payload)} bytes), skipped")
                    continue
                with state_lock:
                    state["latest"] = payload
                    state["seq"] += 1
                new_frame.set()
        except (ConnectionError, socket.timeout, OSError) as why:
            log("mediad connection ended:", why)
        finally:
            with state_lock:
                state["closed"] = True
            new_frame.set()
            worker.join(timeout=2)
            try:
                conn.close()
            except Exception:
                pass

    def handshake(self, conn):
        method, path, headers = read_http_request(conn)
        if method != "GET" or path.split("?", 1)[0] != self.args.path:
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nConnection: close\r\nContent-Length: 0\r\n\r\n")
            raise ConnectionError(f"unexpected request {method} {path}")
        key = headers.get("sec-websocket-key")
        if not key:
            conn.sendall(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\nContent-Length: 0\r\n\r\n")
            raise ConnectionError("missing sec-websocket-key")
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key(key)}\r\n"
            "\r\n"
        )
        conn.sendall(response.encode("ascii"))

    def serve(self):
        host, port = self.args.listen_host, self.args.listen_port
        try:
            listener = socket.create_server((host, port))
        except OSError as why:
            log(f"cannot listen on {host}:{port}:", why)
            return 2
        listener.settimeout(0.5)
        log(f"duck-vision listening on ws://{host}:{port}{self.args.path}")
        log(f"vision endpoint: {self.args.model_url} (interval {self.args.interval}s)")
        if self.args.once:
            deadline = time.time() + self.args.wait
            while time.time() < deadline and not self.stop.is_set():
                try:
                    conn, _addr = listener.accept()
                except socket.timeout:
                    continue
                try:
                    conn.settimeout(10)
                    self.handshake(conn)
                    ok = self.handle_client_once(conn)
                    listener.close()
                    return 0 if ok else 1
                except (ConnectionError, socket.timeout) as why:
                    log("connection failed:", why)
                    try:
                        conn.close()
                    except Exception:
                        pass
            listener.close()
            log(f"no mediad connection within {self.args.wait}s")
            return 2

        threads = []
        try:
            while not self.stop.is_set():
                try:
                    conn, addr = listener.accept()
                except socket.timeout:
                    continue
                try:
                    conn.settimeout(10)
                    self.handshake(conn)
                    conn.settimeout(None)
                except (ConnectionError, socket.timeout) as why:
                    log("handshake failed:", why)
                    try:
                        conn.close()
                    except Exception:
                        pass
                    continue
                thread = threading.Thread(
                    target=self.handle_client, args=(conn, addr), name="duck-vision-client", daemon=True
                )
                thread.start()
                threads.append(thread)
        finally:
            listener.close()
            for thread in threads:
                thread.join(timeout=2)
        return 0


def parse_listen(value):
    if ":" not in value:
        raise argparse.ArgumentTypeError("listen must be HOST:PORT")
    host, port = value.rsplit(":", 1)
    try:
        return host, int(port)
    except ValueError as why:
        raise argparse.ArgumentTypeError("listen port must be a number") from why


def main():
    parser = argparse.ArgumentParser(
        description="Receive mediad --stream-to JPEG frames and analyze them with a local VLM."
    )
    parser.add_argument("--listen", default=os.environ.get("VISION_LISTEN", "127.0.0.1:8765"),
                        help="HOST:PORT for the WebSocket mediad dials (default 127.0.0.1:8765)")
    parser.add_argument("--path", default=os.environ.get("VISION_PATH", "/frames"),
                        help="WebSocket path mediad dials (default /frames)")
    parser.add_argument("--model-url", default=DEFAULT_MODEL_URL,
                        help="OpenAI-compatible base URL (default LLAMA_URL or http://127.0.0.1:8081/v1)")
    parser.add_argument("--model", default=os.environ.get("VISION_MODEL", ""),
                        help="model id to send; empty = first id from /models")
    parser.add_argument("--prompt", default=os.environ.get("VISION_PROMPT", DEFAULT_PROMPT),
                        help="prompt sent with every analyzed frame")
    parser.add_argument("--interval", type=float, default=float(os.environ.get("VISION_INTERVAL", "5")),
                        help="seconds between analyses; newest frame wins (default 5)")
    parser.add_argument("--max-tokens", type=int, default=int(os.environ.get("VISION_MAX_TOKENS", "220")))
    parser.add_argument("--timeout", type=float, default=float(os.environ.get("VISION_TIMEOUT", "180")),
                        help="vision endpoint timeout seconds")
    parser.add_argument("--save-dir", default=os.environ.get("VISION_SAVE_DIR", ""),
                        help="optional directory to save analyzed JPEG frames")
    parser.add_argument("--min-bytes", type=int, default=256,
                        help="drop binary messages smaller than this")
    parser.add_argument("--once", action="store_true",
                        help="analyze the first valid frame, print it, and exit")
    parser.add_argument("--wait", type=float, default=30,
                        help="once mode: seconds to wait for mediad and a frame")
    args = parser.parse_args()

    args.listen_host, args.listen_port = parse_listen(args.listen)
    if not args.path.startswith("/"):
        args.path = "/" + args.path
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    server = Server(args)

    def stop(_signum, _frame):
        server.stop.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    return server.serve()


if __name__ == "__main__":
    raise SystemExit(main())
