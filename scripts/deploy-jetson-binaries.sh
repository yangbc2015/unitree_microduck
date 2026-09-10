#!/usr/bin/env bash
# Build this workspace and install every binary into the board's release tree.
#
# The tree is `/opt/robot/daemon/releases/<version>/bin`, `current` symlinks it, and every unit
# ExecStart's a path under `current`. Nothing here is signed and nothing goes through `updaterd`:
# this is the bench path for a board somebody is working on, and the updater is `updater/` with its
# own design doc.
#
# **All of them or none.** The tree is named for one release and its daemons share one API version.
# Installing only the daemon that was just changed leaves a board where `hello` - answered by
# `updaterd` - and `robot.policies` - answered by `robotd` - disagree with each other, which is
# exactly what `docs/project/jetson-port.md` records as item 12: the console printed
# `api_version: 16, daemon_version: "0.10.0"` for a robot whose `robotd` was a fresh 0.11.0 build.
#
# Seconds before this by hand: `sudo install -m755 target/release/<name> <release>/bin/`, once per
# binary, with whichever ones somebody remembered. That is how a tree named 0.11.0-jetson came to
# hold 0.10.0 binaries.
set -euo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
CURRENT=${CURRENT:-/opt/robot/daemon/current}
RELEASE_DIR=$(readlink -f "$CURRENT")
BIN_DIR="$RELEASE_DIR/bin"
BACKUP_ROOT=${BACKUP_ROOT:-/opt/robot/daemon/backups}
STAMP=$(date +%Y%m%d-%H%M%S)

echo "==> building the workspace (native aarch64; see scripts/… for the toolchain)"
cd "$REPO"
cargo build --release

echo "==> release tree: $BIN_DIR (via $CURRENT)"
[ -d "$BIN_DIR" ] || { echo "no $BIN_DIR: is this a robot?" >&2; exit 1; }

echo "==> backing up to $BACKUP_ROOT/$STAMP-bin"
sudo mkdir -p "$BACKUP_ROOT"
sudo cp -a "$BIN_DIR" "$BACKUP_ROOT/$STAMP-bin"

echo "==> installing"
missing=()
installed=()
# Driven by what the tree already holds, so a binary that is in the tree and not in this build is
# reported rather than silently left at the old release.
for path in "$BIN_DIR"/*; do
    name=$(basename "$path")
    if [ -f "$REPO/target/release/$name" ]; then
        sudo install -m755 "$REPO/target/release/$name" "$path"
        installed+=("$name")
    else
        missing+=("$name")
    fi
done
printf '    installed: %s\n' "${installed[*]:-none}"
[ ${#missing[@]} -eq 0 ] || printf '    NOT in this build (left as they were): %s\n' "${missing[*]}"

echo "==> restarting the enabled units"
for unit in updaterd robotd configd btd mediad padd tofd; do
    if systemctl is-enabled "$unit" >/dev/null 2>&1; then
        sudo systemctl restart "$unit"
        printf '    %-10s %s\n' "$unit" "$(systemctl is-active "$unit")"
    fi
done

echo "==> what the deployed daemons now say (hello is updaterd's answer, by design)"
sudo python3 - <<'PY'
import json, os, socket
# Named rather than globbed: /run holds other daemons' sockets, and a report that waits on seatd
# for five seconds teaches nobody anything.
for path in ("/run/robotd.sock", "/run/updaterd.sock", "/run/configd.sock"):
    if not os.path.exists(path):
        print(f"    {path:24s} (not running)")
        continue
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(5)
            sock.connect(path)
            sock.sendall(b'{"jsonrpc":"2.0","id":1,"method":"hello","params":{"api_version":27}}\n')
            answer = json.loads(sock.recv(65536).decode())
        print(f"    {path:24s} api_version={answer['result']['api_version']} daemon={answer['result']['daemon_version']}")
    except Exception as exc:
        print(f"    {path:24s} {type(exc).__name__}: {exc}")
PY
echo "    camera: $(sudo cat /run/mediad/camera.json 2>/dev/null || echo '(mediad not publishing)')"
