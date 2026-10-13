#!/usr/bin/env python3
import fcntl
import os
import subprocess
import sys
import time

if len(sys.argv) < 3:
    raise SystemExit("usage: spawn_model_serialized.py <delay_seconds> <spawn_model_args...>")

delay = float(sys.argv[1])
lock_path = "/tmp/zzu_fly_spawn_models.lock"
with open(lock_path, "w") as lock_file:
    fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
    time.sleep(delay)
    command = ["/opt/ros/noetic/lib/gazebo_ros/spawn_model"] + sys.argv[2:]
    completed = subprocess.run(command)
    raise SystemExit(completed.returncode)
