#!/usr/bin/env python3
"""Verify /map is populated by scans without referee obstacle parameters."""

import os
from pathlib import Path
import socket
import subprocess
import threading
import time

import rosgraph
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros


ROOT = Path(__file__).resolve().parents[3]


def cell(grid, x, y):
    col = int((x - grid.info.origin.position.x) / grid.info.resolution)
    row = int((y - grid.info.origin.position.y) / grid.info.resolution)
    return grid.data[row * grid.info.width + col]


def main():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    master = subprocess.Popen(["roscore", "-p", str(port)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fuser = None
    stop = threading.Event()
    worker = None
    try:
        for _ in range(100):
            try:
                rosgraph.Master("/lidar_map_fuser_test").getPid()
                break
            except Exception:
                time.sleep(0.1)
        else:
            raise AssertionError("isolated ROS master did not start")
        rospy.init_node("lidar_map_fuser_test", disable_signals=True)
        fuser = subprocess.Popen([str(ROOT / "devel/lib/auto_explore/scan_map_fuser_node"),
                                  "_num_drones:=1"], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.PIPE)
        before = rospy.wait_for_message("/map", OccupancyGrid, timeout=8)
        assert cell(before, -7, -3) == -1
        scan_pub = rospy.Publisher("/typhoon_h480_0/scan_filtered", LaserScan,
                                   queue_size=1)
        broadcaster = tf2_ros.TransformBroadcaster()

        def emit():
            while not stop.is_set() and not rospy.is_shutdown():
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "odom_0"
                transform.child_frame_id = "base_link_0"
                transform.transform.rotation.w = 1.0
                broadcaster.sendTransform(transform)
                scan = LaserScan()
                scan.header.stamp = transform.header.stamp
                scan.header.frame_id = "base_link_0"
                scan.angle_min = 0.0
                scan.angle_max = 0.0
                scan.angle_increment = 1.0
                scan.range_min = 0.1
                scan.range_max = 20.0
                scan.ranges = [10.0]
                scan_pub.publish(scan)
                time.sleep(0.05)

        worker = threading.Thread(target=emit, daemon=True)
        worker.start()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            mapped = rospy.wait_for_message("/map", OccupancyGrid, timeout=2)
            if any(cell(mapped, -7 + dx * 0.1, -3 + dy * 0.1) == 100
                   for dx in range(-1, 2) for dy in range(-1, 2)):
                assert cell(mapped, 20, -25) == -1
                print("PASS: /map starts unknown and contains only observed lidar hits")
                return
        raise AssertionError("laser hit did not reach /map; process=%s" % fuser.poll())
    finally:
        stop.set()
        if worker is not None:
            worker.join(timeout=2)
        if fuser is not None:
            fuser.terminate()
            fuser.wait(timeout=3)
        rospy.signal_shutdown("test finished")
        master.terminate()
        master.wait(timeout=3)


if __name__ == "__main__":
    main()
