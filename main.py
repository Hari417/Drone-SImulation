#!/usr/bin/env python3
"""
main.py

Launcher for multi-drone swarm coordination agents.

Reads a shared YAML fleet configuration and starts one drone_main.py
process per drone, passing each the config file and its own drone_id.

Usage:
    python main.py
"""

import subprocess
import yaml
import os
import sys

# ------------------------------------------------------------------------------
# CONFIG
# ------------------------------------------------------------------------------

# Path to shared YAML config of the team
CONFIG_PATH = "fleet_config.yaml"

if not os.path.isfile(CONFIG_PATH):
    print(f"Error: configuration file '{CONFIG_PATH}' not found.")
    sys.exit(1)

# ------------------------------------------------------------------------------
# LOAD FLEET CONFIG
# ------------------------------------------------------------------------------

with open(CONFIG_PATH, "r") as f:
    cfg = yaml.safe_load(f)

if "drones" not in cfg or not isinstance(cfg["drones"], list):
    print("Invalid fleet_config.yaml format: missing 'drones' list.")
    sys.exit(1)

drone_list = cfg["drones"]

# ------------------------------------------------------------------------------
# LAUNCH SWARM AGENTS
# ------------------------------------------------------------------------------

processes = []

print("Launching drone agents...")

for drone in drone_list:
    drone_id = drone.get("drone_id")
    if not drone_id:
        print("Warning: drone entry missing 'drone_id'; skipping")
        continue

    print(f"  > Starting agent for {drone_id}")

    # Build the command:
    #   python drone_main.py --config <CONFIG_PATH> --id <drone_id>
    cmd = [
        sys.executable,      # ensures same Python interpreter
        "drone_main.py",
        "--config", CONFIG_PATH,
        "--id", drone_id
    ]

    # Start the process
    try:
        p = subprocess.Popen(cmd)
        processes.append((drone_id, p))
    except Exception as e:
        print(f"Error launching agent for {drone_id}: {e}")

# ------------------------------------------------------------------------------
# WAIT FOR ALL AGENTS
# ------------------------------------------------------------------------------

try:
    for drone_id, p in processes:
        print(f"Waiting for {drone_id} to finish...")
        p.wait()
        print(f"{drone_id} exited with code {p.returncode}")
except KeyboardInterrupt:
    print("Interrupted. Terminating agent processes...")
    for drone_id, p in processes:
        try:
            p.terminate()
        except Exception:
            pass
    print("Termination complete.")

print("All drone agents have exited.")
