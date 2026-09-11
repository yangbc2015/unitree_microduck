#!/usr/bin/env bash
# Deploy the locally-compiled microduck daemons onto this Jetson as a systemd skeleton.
#
# Faithful to the upstream install layout (/opt/robot/daemon/current -> releases/<ver>)
# but uses the binaries we built in target/release instead of a signed release artifact.
# Non-destructive: refuses to overwrite an existing /etc/robot/robotd.toml, and only
# creates systemd units / sysusers that are absent.
#
# What this does NOT do (Jetson has no robot hardware):
#   - robotd will run its IPC but never find a motor bus -> "degraded" health, by design
#   - btd / mediad / padd / tofd will be *installed* but NOT enabled (no BT pad / camera
#     pipeline / ToF on this box unless present); enable them deliberately later
set -euo pipefail

REPO="$HOME/Desktop/microduck"
REL="0.10.0-jetson"
DAEMON_DIR="/opt/robot/daemon"
REL_DIR="$DAEMON_DIR/releases/$REL"
BIN_SRC="$REPO/target/release"

# binaries shipped as daemons/tools (actual [[bin]] names in target/release)
BINS=(robotd robotctl btd configd mediad padd updaterd tofd xtask)
SERVICE_UNITS=(robotd btd configd mediad padd updaterd tofd)
SYSUSERS=(btd mediad padd tofd)

echo "==> 1/7 creating release tree $REL_DIR/bin"
sudo mkdir -p "$REL_DIR/bin"
for b in "${BINS[@]}"; do
  if [ -x "$BIN_SRC/$b" ]; then
    sudo cp -f "$BIN_SRC/$b" "$REL_DIR/bin/$b"
    echo "    copied $b ($(du -h "$REL_DIR/bin/$b" | cut -f1))"
  else
    echo "    SKIP $b (not built)"
  fi
done
# tofd lives in tof/ crate -> binary name is tofd
if [ -x "$BIN_SRC/tofd" ]; then sudo cp -f "$BIN_SRC/tofd" "$REL_DIR/bin/tofd"; fi

echo "==> 2/7 symlinking current -> releases/$REL"
sudo ln -sfn "$REL_DIR" "$DAEMON_DIR/current"

echo "==> 3/7 installing systemd units"
for u in "${SERVICE_UNITS[@]}"; do
  src=$(find "$REPO" -path '*/systemd/'"$u"'.service' -not -path '*/target/*' | head -1)
  if [ -n "$src" ]; then
    sudo cp -f "$src" "/etc/systemd/system/$u.service"
    echo "    $u.service"
  fi
done

# Board delta that is not code: mediad's unit is upstream's, and on this board it needs the Argus
# socket visible inside its private /tmp (see the file, and docs/project/jetson-port.md). A drop-in
# rather than an edit, so mediad/systemd/mediad.service stays byte-for-byte upstream's.
if [ -f "$REPO/deploy/jetson/10-argus-socket.conf" ]; then
  sudo mkdir -p /etc/systemd/system/mediad.service.d
  sudo cp -f "$REPO/deploy/jetson/10-argus-socket.conf" /etc/systemd/system/mediad.service.d/
  echo "    mediad.service.d/10-argus-socket.conf"
fi

# The second board delta of the same kind: `--rotate 0` because the camera here is mounted square to
# the world and `mediad`'s default is the Radxa's quarter turn (see the file). Same reason for a
# drop-in - the unit stays byte-for-byte upstream's.
if [ -f "$REPO/deploy/jetson/20-mount.conf" ]; then
  sudo mkdir -p /etc/systemd/system/mediad.service.d
  sudo cp -f "$REPO/deploy/jetson/20-mount.conf" /etc/systemd/system/mediad.service.d/
  echo "    mediad.service.d/20-mount.conf"
fi

# The third board delta of the same kind: the frame stream the local VLM reads, off the same tee the
# console's video comes from (see the file). It repeats `--rotate 0`, because both drop-ins set
# `ExecStart` and systemd honours the one in the file that sorts last.
if [ -f "$REPO/deploy/jetson/30-stream.conf" ]; then
  sudo mkdir -p /etc/systemd/system/mediad.service.d
  sudo cp -f "$REPO/deploy/jetson/30-stream.conf" /etc/systemd/system/mediad.service.d/
  echo "    mediad.service.d/30-stream.conf"
fi

echo "==> 4/7 creating robot group + daemon users (sysusers)"
sudo cp -f "$REPO/updater/systemd/sysusers.d/robot.conf" /usr/lib/sysusers.d/robot.conf
for s in "${SYSUSERS[@]}"; do
  src=$(find "$REPO" -path '*/sysusers.d/'"$s"'.conf' -not -path '*/target/*' | head -1)
  [ -n "$src" ] && sudo cp -f "$src" "/usr/lib/sysusers.d/$s.conf"
done
sudo systemd-sysusers || true
getent group robot >/dev/null && echo "    group robot OK" || echo "    !! group robot MISSING"

echo "==> 5/7 installing /etc/robot config"
sudo mkdir -p /etc/robot
if [ ! -e /etc/robot/robotd.toml ]; then
  sudo cp "$REPO/deploy/robotd.toml" /etc/robot/robotd.toml
  echo "    robotd.toml (fresh)"
else
  echo "    robotd.toml exists, left untouched"
fi
sudo cp -f "$REPO/deploy/updater.toml" /etc/robot/updater.toml
sudo mkdir -p /etc/robot/trusted_keys
sudo cp -f "$REPO/deploy/trusted_keys/"*.pub /etc/robot/trusted_keys/ 2>/dev/null || true
echo "    updater.toml + trusted_keys"

echo "==> 6/7 state dir + journald drop-in"
sudo mkdir -p /var/lib/robot/updater
sudo mkdir -p /etc/systemd/journald.conf.d
sudo cp -f "$REPO/deploy/journald.conf.d/10-robot.conf" /etc/systemd/journald.conf.d/10-robot.conf
sudo systemctl restart systemd-journald 2>/dev/null || true
echo "    /var/lib/robot/updater + journald drop-in"

echo "==> 7/7 daemon-reload"
sudo systemctl daemon-reload
echo
echo "DONE. Layout:"
ls -l "$DAEMON_DIR/current" | sed 's/^/    /'
echo "Next step suggestion:"
echo "    sudo systemctl enable --now robotd        # IPC comes up; health=degraded (no motor bus)"
echo "    sudo systemctl status robotd"
