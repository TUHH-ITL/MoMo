#!/usr/bin/env bash
# Grants device_container_node the Linux capabilities it needs to manage
# CAN controller state (e.g. bus-off recovery) without running as root.
# Without this, the node aborts on startup with:
#   terminate called after throwing an instance of 'std::system_error'
#     what():  CanController: Operation not permitted
#
# Capabilities live on the file, not the package, so colcon reinstalling this
# binary on every build wipes them. Re-applied here at install time.
#
# `--symlink-install` makes the install-tree path a symlink to the build-tree
# binary; `setcap` refuses to operate on a symlink directly ("Invalid file
# for capability operation"), so resolve to the real file first.
#
# Requires a NOPASSWD sudoers rule for this exact resolved command (see
# mecanum_maxon_control/README.md > "CAN capabilities"). Without it this
# script fails quietly and prints a reminder instead of breaking the build.
set -euo pipefail
BIN="$(readlink -f "$1")"

if sudo -n setcap cap_net_admin,cap_net_raw+ep "$BIN" 2>/dev/null; then
  exit 0
fi

cat >&2 <<EOF
WARNING: could not set CAN capabilities on:
  $BIN
device_container_node will fail with "CanController: Operation not
permitted" until this is fixed. Run once:
  sudo setcap cap_net_admin,cap_net_raw+ep "$BIN"
or install the NOPASSWD sudoers rule described in
mecanum_maxon_control/README.md > "CAN capabilities" so future builds
apply it automatically.
EOF
