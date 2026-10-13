#!/usr/bin/env python3
import math
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler


class VelocityContinuityTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._commands = []
        vehicle_index = {
            "test_forward_speed_brakes_quickly_when_badly_misaligned": 0,
            "test_forward_speed_is_continuous_across_alignment_threshold": 1,
            "test_forward_speed_ramps_up_after_alignment": 2,
        }[self._testMethodName]
        namespace = "/typhoon_h480_{}".format(vehicle_index)
        self._pose_pub = rospy.Publisher(
            namespace + "/global_pose", PoseStamped, queue_size=1, latch=True
        )
        self._goal_pub = rospy.Publisher(
            namespace + "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )
        self._cmd_sub = rospy.Subscriber(
            namespace + "/mux_inputs/navigator/cmd_vel", Twist, self._command_callback
        )
        self._wait_for_connections()

    def _wait_for_connections(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self._pose_pub.get_num_connections() and self._goal_pub.get_num_connections():
                return
            rate.sleep()
        self.fail("navigator did not subscribe to pose and goal topics")

    def _command_callback(self, msg):
        with self._lock:
            self._commands.append(msg.linear.x)

    @staticmethod
    def _pose(yaw_degrees):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0.0, 0.0, math.radians(yaw_degrees))
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        return msg

    @staticmethod
    def _goal():
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = 100.0
        msg.pose.orientation.w = 1.0
        return msg

    def _latest_command_after_pose(self, yaw_degrees):
        with self._lock:
            self._commands.clear()
        self._pose_pub.publish(self._pose(yaw_degrees))
        rospy.sleep(0.2)
        deadline = rospy.Time.now() + rospy.Duration(1.0)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._lock:
                if self._commands:
                    return self._commands[-1]
            rate.sleep()
        self.fail("navigator did not publish a command after pose update")

    def test_forward_speed_is_continuous_across_alignment_threshold(self):
        self._pose_pub.publish(self._pose(29.0))
        self._goal_pub.publish(self._goal())
        rospy.sleep(0.3)

        before = self._latest_command_after_pose(29.0)
        after = self._latest_command_after_pose(31.0)

        self.assertLess(
            abs(before - after),
            0.5,
            "crossing the yaw-alignment threshold must not step the forward command",
        )

    def test_forward_speed_brakes_quickly_when_badly_misaligned(self):
        self._pose_pub.publish(self._pose(0.0))
        self._goal_pub.publish(self._goal())
        rospy.sleep(3.5)

        before = self._latest_command_after_pose(0.0)
        self.assertGreater(before, 5.5)

        with self._lock:
            self._commands.clear()
        self._pose_pub.publish(self._pose(90.0))
        rospy.sleep(0.5)
        with self._lock:
            after = self._commands[-1] if self._commands else None

        self.assertIsNotNone(after)
        self.assertLessEqual(
            after,
            0.601,
            "large yaw errors must use the configured braking limit",
        )

    def test_forward_speed_ramps_up_after_alignment(self):
        self._pose_pub.publish(self._pose(31.0))
        self._goal_pub.publish(self._goal())
        rospy.sleep(0.5)

        command = self._latest_command_after_pose(0.0)

        self.assertGreater(command, 0.0)
        self.assertLessEqual(
            command,
            0.6,
            "forward speed must respect the configured acceleration limit",
        )


if __name__ == "__main__":
    rospy.init_node("test_velocity_continuity")
    rostest.rosrun("simple_navigator", "velocity_continuity", VelocityContinuityTest)
