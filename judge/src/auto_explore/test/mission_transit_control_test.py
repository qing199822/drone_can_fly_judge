#!/usr/bin/env python3
"""Check that an out-of-region UAV keeps its TRANSIT controller active."""

import math
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


VEHICLE = "mission_transit_probe_4"


def main():
    rospy.init_node("mission_transit_control_test")
    pose_pub = rospy.Publisher("/%s/mavros/local_position/pose" % VEHICLE,
                               PoseStamped, queue_size=2)
    scan_pub = rospy.Publisher("/%s/scan_filtered" % VEHICLE,
                               LaserScan, queue_size=2)
    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    mapped = OccupancyGrid()
    mapped.header.frame_id = "map"
    mapped.info.resolution = 1.0
    mapped.info.width = 230
    mapped.info.height = 120
    mapped.info.origin.position.x = -90.0
    mapped.info.origin.position.y = -60.0
    mapped.info.origin.orientation.w = 1.0
    mapped.data = [-1] * (mapped.info.width * mapped.info.height)
    map_pub.publish(mapped)
    diagnostics = []
    commands = []
    rospy.Subscriber("/%s/area_mission/diag" % VEHICLE, String,
                     lambda msg: diagnostics.append(msg.data), queue_size=20)
    rospy.Subscriber("/%s/mux_inputs/navigator/cmd_vel" % VEHICLE, Twist,
                     lambda msg: commands.append((msg.linear.x, msg.linear.y)), queue_size=20)
    stop = threading.Event()

    def publish_sensors():
        while not stop.is_set() and not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.z = 4.5
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)

            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / 640.0
            scan.range_min = 0.08
            scan.range_max = 20.0
            scan.ranges = [12.0] * 640
            scan_pub.publish(scan)
            time.sleep(0.05)

    thread = threading.Thread(target=publish_sensors, daemon=True)
    thread.start()
    node = None
    try:
        time.sleep(0.5)
        node = subprocess.Popen([
            "rosrun", "auto_explore", "area_mission_node", "__ns:=" + VEHICLE,
            "_vehicle:=" + VEHICLE, "_target_altitude:=4.5", "_diag_rate:=20.0",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            transit = [msg for msg in diagnostics if "mode=3 " in msg and "stop=0 " in msg]
            if len(transit) >= 3 and any(vx > 0.1 for vx, _ in commands):
                assert not any("mode=2 " in msg for msg in diagnostics), diagnostics[-1]
                for y in range(60, 68):
                    for x in range(69, 77):
                        if not (x == 73 and y == 63):
                            mapped.data[y * mapped.info.width + x] = 100
                map_pub.publish(mapped)
                time.sleep(0.4)
                blocked = len(commands)
                time.sleep(0.8)
                assert commands[blocked:], "no commands after static obstacle update"
                assert all(abs(vx) < 0.01 and abs(vy) < 0.01
                           for vx, vy in commands[blocked:]), commands[blocked:]
                print("PASS: TRANSIT advances into unknown space and stops at mapped lidar obstacles")
                return
            time.sleep(0.05)
        raise AssertionError("TRANSIT did not publish forward commands: %s" %
                             (diagnostics[-1] if diagnostics else "no diagnostics"))
    finally:
        stop.set()
        thread.join(timeout=2.0)
        if node is not None:
            node.terminate()
            try:
                node.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                node.kill()
                node.wait()


if __name__ == "__main__":
    main()
