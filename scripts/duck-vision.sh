#!/usr/bin/env bash
# Start microduck's camera daemon and the local AI-vision bridge together.
#
# This is the "duck gets eyes" entry point: mediad captures the head camera and
# dials a local WebSocket with downscaled JPEG frames; scripts/duck-vision.py
# answers that socket and sends the newest frame every VISION_INTERVAL seconds
# to an OpenAI-compatible multimodal endpoint (llama.cpp by default).
#
# Everything is configurable from the environment, so this does not depend on
# the current machine's setup beyond a mediad binary and python3:
#
#   MEDIAD_BIN        default: target/release/mediad, then /opt/robot/daemon/current/bin/mediad
#   VISION_LISTEN     default: 127.0.0.1:8765   (where duck-vision.py listens)
#   VISION_PATH       default: /frames          (WebSocket path mediad dials)
#   LLAMA_URL         default: http://127.0.0.1:8081/v1  (any OpenAI-compatible VLM endpoint)
#   VISION_MODEL      default: first id from $LLAMA_URL/models
#   VISION_PROMPT     default: built-in Chinese scene-description prompt
#   VISION_INTERVAL   default: 5                (seconds between analyses)
#   VISION_SAVE_DIR   default: unset            (set to save analyzed JPEGs)
#   VISION_FPS        default: 1                (frames mediad pushes; analysis uses newest)
#   VISION_LONGEST    default: 640              (longest edge of pushed JPEG)
#   VISION_QUALITY    default: 70               (JPEG quality)
#   MEDIAD_ROTATE     default: unset            (set 0 on the Jetson Arducam; real duck keeps mediad's 90)
#   MEDIAD_CSI_PORT   default: unset            (set 0/1 to pick the CSI port)
#   MEDIAD_ARGS       default: unset            (extra args for mediad, word-split)
#
# Ctrl-C (or mediad exiting) stops both halves.

set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MEDIAD_BIN=${MEDIAD_BIN:-$ROOT/target/release/mediad}
if [[ ! -x $MEDIAD_BIN && -x /opt/robot/daemon/current/bin/mediad ]]; then
    MEDIAD_BIN=/opt/robot/daemon/current/bin/mediad
fi
if [[ ! -x $MEDIAD_BIN ]]; then
    echo "error: no mediad binary at $MEDIAD_BIN" >&2
    echo "build it first: cargo build --release -p mediad   (or set MEDIAD_BIN)" >&2
    exit 1
fi
if ! "$MEDIAD_BIN" --help 2>&1 | grep -q -- '--stream-to'; then
    echo "error: $MEDIAD_BIN has no --stream-to; rebuild mediad from this tree" >&2
    echo "       cargo build --release -p mediad" >&2
    exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
    echo "error: python3 is required for scripts/duck-vision.py" >&2
    exit 1
fi

LISTEN=${VISION_LISTEN:-127.0.0.1:8765}
WS_PATH=${VISION_PATH:-/frames}
LLAMA_URL=${LLAMA_URL:-http://127.0.0.1:8081/v1}
VISION_INTERVAL=${VISION_INTERVAL:-5}
VISION_FPS=${VISION_FPS:-1}
VISION_LONGEST=${VISION_LONGEST:-640}
VISION_QUALITY=${VISION_QUALITY:-70}
EXTRA_MEDIAD_ARGS=${MEDIAD_ARGS:-}

IFS=: read -r listen_host listen_port <<< "$LISTEN"
if [[ -z ${listen_host:-} || -z ${listen_port:-} ]]; then
    echo "error: VISION_LISTEN must be HOST:PORT, got '$LISTEN'" >&2
    exit 1
fi
dial_host=$listen_host
if [[ $dial_host == "0.0.0.0" || $dial_host == "::" ]]; then
    dial_host=127.0.0.1
fi
WS_URL="ws://$dial_host:$listen_port$WS_PATH"

if command -v curl >/dev/null 2>&1; then
    if ! curl --noproxy '*' -fsS --max-time 2 "$LLAMA_URL/models" >/dev/null 2>&1; then
        echo "warn: vision endpoint $LLAMA_URL is not answering; analyses will fail until it is up" >&2
    fi
fi

vision_args=(--listen "$LISTEN" --path "$WS_PATH" --model-url "$LLAMA_URL" --interval "$VISION_INTERVAL")
if [[ -n ${VISION_MODEL:-} ]]; then
    vision_args+=(--model "$VISION_MODEL")
fi
if [[ -n ${VISION_PROMPT:-} ]]; then
    vision_args+=(--prompt "$VISION_PROMPT")
fi
if [[ -n ${VISION_SAVE_DIR:-} ]]; then
    vision_args+=(--save-dir "$VISION_SAVE_DIR")
fi

mediad_args=(--stream-to "$WS_URL" --stream-fps "$VISION_FPS" --stream-longest "$VISION_LONGEST" --stream-quality "$VISION_QUALITY")
if [[ -n ${MEDIAD_ROTATE:-} ]]; then
    mediad_args+=(--rotate "$MEDIAD_ROTATE")
fi
if [[ -n ${MEDIAD_CSI_PORT:-} ]]; then
    mediad_args+=(--csi-port "$MEDIAD_CSI_PORT")
fi
if [[ -n $EXTRA_MEDIAD_ARGS ]]; then
    read -r -a extra_args <<< "$EXTRA_MEDIAD_ARGS"
    mediad_args+=("${extra_args[@]}")
fi

vision_pid=""
mediad_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ -n $mediad_pid ]]; then
        kill "$mediad_pid" 2>/dev/null || true
    fi
    if [[ -n $vision_pid ]]; then
        kill "$vision_pid" 2>/dev/null || true
    fi
    wait $mediad_pid $vision_pid 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "duck-vision: $LLAMA_URL  (every ${VISION_INTERVAL}s, ws://$LISTEN$WS_PATH)"
python3 "$ROOT/scripts/duck-vision.py" "${vision_args[@]}" &
vision_pid=$!

echo "mediad: $MEDIAD_BIN -> $WS_URL (${VISION_FPS} fps, longest ${VISION_LONGEST}, q${VISION_QUALITY})"
"$MEDIAD_BIN" "${mediad_args[@]}" &
mediad_pid=$!

wait "$mediad_pid"
