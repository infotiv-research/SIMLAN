#!/usr/bin/env bash
# Build and install the actor-collision Gazebo plugin
# and expose it to Gazebo via a symlink, not an env-var.

set -e  # Exit on first error

# ── Configurable paths ──────────────────────────────────────────────────
PLUGIN_WORKDIR="/opt/gazebo_plugins"
PLUGIN_REPO="https://github.com/JiangweiNEU/actor_collisions.git"
PLUGIN_NAME="actor_collisions"
GAZEBO_SYSTEM_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
PLUGIN_LIBRARY="libActorCollisionsPlugin.so"

# ── Remember where we began ─────────────────────────────────────────────
ORIG_DIR="$(pwd)"

# ── Fetch, build, and install the plugin ────────────────────────────────
sudo mkdir -p "$PLUGIN_WORKDIR"
cd "$PLUGIN_WORKDIR"

if [ ! -d "$PLUGIN_NAME" ]; then
  sudo git clone "$PLUGIN_REPO"
fi

cd "$PLUGIN_NAME"
mkdir -p build
cd build

cmake ..
make -j"$(nproc)"

# ── Make the plugin visible to Gazebo via a symlink ─────────────────────
sudo ln -sf "$(pwd)/${PLUGIN_LIBRARY}" \
  "${GAZEBO_SYSTEM_PLUGIN_DIR}/${PLUGIN_LIBRARY}"

echo "Symlink created:"
ls -l "${GAZEBO_SYSTEM_PLUGIN_DIR}/${PLUGIN_LIBRARY}"

# ── Return to the directory we started from ─────────────────────────────
cd "$ORIG_DIR"
