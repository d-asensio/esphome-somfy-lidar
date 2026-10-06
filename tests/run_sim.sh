#!/usr/bin/env bash
# Builds the awning components against the installed ESPHome package for the
# host platform and runs the closed-loop simulation (takes about 3 minutes).
# Requires: pip install esphome, g++ with C++20.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
pkg="$(python3 -c 'import esphome, os; print(os.path.dirname(esphome.__file__))')"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT

mkdir -p "$build/esphome/components"
cp -r "$pkg/core" "$build/esphome/core"
for c in host sensor cover button remote_base binary_sensor cc1101 spi i2c; do
  cp -r "$pkg/components/$c" "$build/esphome/components/$c"
done
cp -r "$here/../components/somfy_lidar" "$build/esphome/components/"
cat > "$build/esphome/core/defines.h" <<'DEF'
#pragma once
#include "esphome/core/macros.h"
#define ESPHOME_BOARD "host"
#define ESPHOME_VARIANT "host"
#define ESPHOME_COMPONENT_COUNT 4
#define ESPHOME_LOOP_TASK_STACK_SIZE 8192
#define ESPHOME_LOG_MAX_LISTENERS 1
#define ESPHOME_THREAD_MULTI_ATOMICS
#define USE_ESPHOME_HOST_MAC_ADDRESS {0x02, 0x00, 0x00, 0x00, 0x00, 0x01}
DEF

cd "$build"
g++ -std=gnu++20 -O1 -DUSE_HOST -I. -o awning_sim \
  "$here/awning_sim.cpp" "$here/stubs.cpp" \
  esphome/components/somfy_lidar/somfy_rts.cpp \
  esphome/components/somfy_lidar/somfy_awning.cpp \
  esphome/components/sensor/*.cpp esphome/components/cover/*.cpp \
  esphome/components/button/button.cpp \
  esphome/core/*.cpp esphome/core/wake/wake_host.cpp esphome/components/host/*.cpp
./awning_sim
