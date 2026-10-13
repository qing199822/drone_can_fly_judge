#!/usr/bin/env python3
"""Launch rviz with the auto_explore config, rewriting the LaserScan display
topic to the requested vehicle's /<vehicle>/scan_filtered (the self-filtered
scan consumed by slam_toolbox and explore_node).

Usage: launch_rviz.py [vehicle] [template.rviz]
"""
import os
import sys
import tempfile

vehicle = sys.argv[1] if len(sys.argv) > 1 else "typhoon_h480_0"

template = None
if len(sys.argv) > 2:
    template = sys.argv[2]
else:
    import subprocess
    template = (subprocess.check_output(["rospack", "find", "auto_explore"])
                .decode().strip() + "/explore.rviz")

with open(template, "r") as f:
    cfg = f.read().replace("/typhoon_h480_0/scan", "/%s/scan" % vehicle)

out = os.path.join(tempfile.gettempdir(), "auto_explore_%s.rviz" % vehicle)
with open(out, "w") as f:
    f.write(cfg)

os.execvp("rviz", ["rviz", "-d", out])
