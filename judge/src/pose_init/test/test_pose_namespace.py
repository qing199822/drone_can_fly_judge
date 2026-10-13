#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PoseNamespaceDataflowTest(unittest.TestCase):
    def test_local_pose_is_republished_without_feeding_mavros_vision_input(self):
        output_topic = "/typhoon_h480_0/global_pose"
        received = []
        rospy.Subscriber(output_topic, PoseStamped, received.append, queue_size=1)
        publisher = rospy.Publisher(
            "/typhoon_h480_0/mavros/local_position/pose", PoseStamped, queue_size=1
        )

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        message = PoseStamped()
        message.pose.position.x = 2.0
        message.pose.position.y = 4.0
        message.pose.position.z = 1.0
        message.pose.orientation.w = 1.0

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline and not received:
            message.header.stamp = rospy.Time.now()
            publisher.publish(message)
            rate.sleep()

        self.assertTrue(received, "pose transformer did not publish the global pose")
        self.assertAlmostEqual(received[-1].pose.position.x, -15.0)
        self.assertAlmostEqual(received[-1].pose.position.y, 1.0)
        self.assertAlmostEqual(received[-1].pose.position.z, 1.0)

        publishers, _, _ = rospy.get_master().getSystemState()[2]
        publisher_map = dict(publishers)
        self.assertNotIn(
            "/typhoon_h480_0/mavros/vision_pose/pose",
            publisher_map,
            "the coordinate transformer must not publish to MAVROS external-vision input",
        )

    def test_local_odometry_is_republished_without_using_mavros_input_topics(self):
        received = []
        rospy.Subscriber(
            "/typhoon_h480_0/global_odom", Odometry, received.append, queue_size=1
        )
        publisher = rospy.Publisher(
            "/typhoon_h480_0/mavros/local_position/odom", Odometry, queue_size=1
        )

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        message = Odometry()
        message.pose.pose.position.x = 2.0
        message.pose.pose.position.y = 4.0
        message.pose.pose.position.z = 1.0
        message.pose.pose.orientation.w = 1.0

        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline and not received:
            message.header.stamp = rospy.Time.now()
            publisher.publish(message)
            rate.sleep()

        self.assertTrue(received, "pose transformer did not publish global odometry")
        self.assertAlmostEqual(received[-1].pose.pose.position.x, -15.0)
        self.assertAlmostEqual(received[-1].pose.pose.position.y, 1.0)
        self.assertAlmostEqual(received[-1].pose.pose.position.z, 1.0)

        publishers, _, _ = rospy.get_master().getSystemState()[2]
        publisher_map = dict(publishers)
        self.assertNotIn(
            "/typhoon_h480_0/mavros/vision_odom/odom",
            publisher_map,
            "the coordinate transformer must not publish to MAVROS odometry input",
        )


if __name__ == "__main__":
    rospy.init_node("pose_namespace_dataflow_test")
    rostest.rosrun("pose_init", "pose_namespace_dataflow", PoseNamespaceDataflowTest)
