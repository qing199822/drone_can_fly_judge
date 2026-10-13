#!/usr/bin/env python3
"""Exercise lateral recovery and bad-EKF altitude handling without Gazebo.

Run against an isolated roscore with /use_sim_time unset.
"""

import math
import subprocess
import sys
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


VEHICLE = "mission_safety_probe_0"


def fields(message):
    result = {}
    for token in message.data.split():
        key, sep, value = token.partition("=")
        if sep:
            result[key] = value
    return result


def wait_for(samples, start, predicate, label, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(predicate(sample) for sample in samples[start:]):
            print("PASS:", label)
            return
        time.sleep(0.05)
    raise AssertionError("%s; latest=%s" % (label, samples[-1] if samples else None))


def main():
    rospy.init_node("mission_safety_fallback_test")
    pose_pub = rospy.Publisher("/%s/mavros/local_position/pose" % VEHICLE,
                               PoseStamped, queue_size=2)
    scan_pub = rospy.Publisher("/%s/scan_filtered" % VEHICLE, LaserScan, queue_size=2)
    truth_pub = rospy.Publisher("/gazebo/model_states", ModelStates, queue_size=2)
    samples = []
    rospy.Subscriber("/%s/area_mission/diag" % VEHICLE, String,
                     lambda msg: samples.append(fields(msg)), queue_size=50)

    state = {"front": 2.5, "side": 8.0, "pose_z": 4.5, "truth_z": 4.5,
             "pose_x": -18.0, "moving": True}
    lock = threading.Lock()
    stop = threading.Event()

    def publish():
        while not stop.is_set() and not rospy.is_shutdown():
            with lock:
                if state["moving"]:
                    state["pose_x"] += 0.04
                front, side = state["front"], state["side"]
                pose_z, truth_z = state["pose_z"], state["truth_z"]
                pose_x = state["pose_x"]
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = pose_x
            pose.pose.position.y = -32.0
            pose.pose.position.z = pose_z
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)

            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / 640.0
            scan.range_min = 0.08
            scan.range_max = 20.0
            scan.ranges = []
            for i in range(640):
                degrees = abs(math.degrees(scan.angle_min + i * scan.angle_increment))
                scan.ranges.append(front if degrees <= 15.0 else
                                   side if 70.0 <= degrees <= 110.0 else 12.0)
            scan_pub.publish(scan)

            truth = ModelStates()
            truth.name = [VEHICLE]
            truth_pose = Pose()
            truth_pose.position.z = truth_z
            truth.pose = [truth_pose]
            truth_pub.publish(truth)
            time.sleep(0.05)

    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    node = None
    try:
        time.sleep(0.5)
        node = subprocess.Popen([
            "rosrun", "auto_explore", "area_mission_node", "__ns:=" + VEHICLE,
            "_vehicle:=" + VEHICLE, "_target_altitude:=4.5", "_diag_rate:=20.0",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        wait_for(samples, 0, lambda s: s.get("mode") == "2", "mission entered patrol", 10.0)
        start = len(samples)
        time.sleep(3.0)
        if any(s.get("recovery_state") != "0" for s in samples[start:]):
            raise AssertionError("recovery preempted progress under MPPI")
        print("PASS: continued progress keeps MPPI in control")
        with lock:
            state["moving"] = False
        wait_for(samples, len(samples),
                 lambda s: abs(float(s.get("v_lat_pub", 0))) > 0.2,
                 "recovery moves into clear side after stall", 8.0)

        start = len(samples)
        with lock:
            state["side"] = 0.8
        wait_for(samples, start,
                 lambda s: s.get("safety_reason") == "no_escape" and
                 s.get("stop") == "11" and
                 abs(float(s.get("v_lat_pub", 1))) < 0.01 and
                 abs(float(s.get("v_pub", 1))) < 0.01,
                 "side obstacle cancels horizontal recovery")

        start = len(samples)
        with lock:
            state["front"] = 12.0
            state["pose_z"] = -10.0
        wait_for(samples, start,
                 lambda s: s.get("abnormal_pose") == "3" and s.get("stop") == "3" and
                 abs(float(s.get("v_pub", 1))) < 0.01 and
                 abs(float(s.get("v_lat_pub", 1))) < 0.01,
                 "invalid EKF height stops horizontal motion")

        start = len(samples)
        with lock:
            state["truth_z"] = 3.0
        wait_for(samples, start,
                 lambda s: s.get("abnormal_pose") == "3" and
                 float(s.get("vz_pub", 0)) > 0.1,
                 "fresh truth height commands climb")

        start = len(samples)
        with lock:
            state["truth_z"] = 5.5
        wait_for(samples, start,
                 lambda s: s.get("abnormal_pose") == "3" and
                 float(s.get("vz_pub", 0)) < -0.1,
                 "fresh truth height commands descent above ceiling")
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
    try:
        main()
    except AssertionError as error:
        print("FAIL:", error, file=sys.stderr)
        sys.exit(1)
