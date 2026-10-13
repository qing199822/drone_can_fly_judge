#!/usr/bin/env python3
"""Focused checks for the referee-side actor controller and map inputs."""

import ast
import contextlib
import io
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, Twist
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "XTDrone" / "robocup"))
sys.path.insert(0, str(ROOT / "tools"))

from control_actor import ControlActor
from prepare_actor_control_map import load_boxes, obstacle_points


class ActorControlTest(unittest.TestCase):
    def test_fixed_and_random_world_inputs(self):
        fixed_boxes = load_boxes(ROOT / "waypoint/world/base0.world")
        self.assertGreater(len(fixed_boxes), 10)
        self.assertIn((-49.5, -50.0), obstacle_points(fixed_boxes))

        with tempfile.TemporaryDirectory() as temp_dir:
            black_box = Path(temp_dir) / "black_box.txt"
            black_box.write_text(repr([[[1.0, 3.0], [2.0, 4.0]]]))
            boxes = load_boxes(ROOT / "waypoint/world/base0.world", black_box)
            self.assertEqual(boxes, ast.literal_eval(black_box.read_text()))
            self.assertIn((1.5, 2.5), obstacle_points(boxes))

    def test_controller_avoids_nearby_person(self):
        actor = ControlActor.__new__(ControlActor)
        actor.id = "0"
        actor.actor_num = 6
        actor.avoid = ActorMotion(x=10.0, y=0.0, v=1.0)
        actor.current_pose = Point(x=0.0, y=0.0)
        actor.actor_positions = {"actor_0": actor.current_pose,
                                 "actor_1": Point(x=1.0, y=0.0)}
        actor.uav_positions = {}
        actor.actor_avoidance_radius = 4.0
        actor.actor_min_distance = 2.0
        actor.uav_safety_radius = 7.0
        actor.separation_speed = 2.0
        actor.x_min, actor.x_max = -50.0, 130.0
        actor.y_min, actor.y_max = -50.0, 50.0

        command = actor.build_motion_command()
        self.assertLess(command.x, 0.0)
        self.assertEqual(command.v, 2.0)

    def test_controller_starts_with_round_map(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            Path(temp_dir, "black_box.txt").write_text("[[[1, 3], [2, 4]]]\n")
            Path(temp_dir, "obstacle.txt").write_text("1.5 2.5\n2.5 3.5\n")
            previous_dir = os.getcwd()
            try:
                os.chdir(temp_dir)
                with mock.patch("control_actor.rospy.init_node"), \
                     mock.patch("control_actor.rospy.Publisher") as publisher, \
                     mock.patch("control_actor.rospy.Subscriber"), \
                     mock.patch("control_actor.rospy.ServiceProxy"), \
                     contextlib.redirect_stdout(io.StringIO()):
                    actor = ControlActor("0")
                self.assertEqual(actor.black_box, [[[1, 3], [2, 4]]])
                self.assertEqual(len(actor.Obstacleavoid.obstList), 2)
                self.assertEqual(publisher.call_args_list[0].args[0],
                                 "/actor_0/cmd_motion")
            finally:
                os.chdir(previous_dir)

    def test_path_clearance_checks_entire_segment(self):
        actor = ControlActor.__new__(ControlActor)
        actor.current_pose = Point(x=0.0, y=0.0)
        actor.obstacle_points = np.asarray([(5.0, 1.0), (20.0, 0.0)])
        self.assertTrue(actor.path_blocked(10.0, 0.0))
        self.assertFalse(actor.path_blocked(10.0, 5.0, clearance=0.5))

    def test_model_state_index_cache_updates_after_deletion(self):
        actor = ControlActor.__new__(ControlActor)
        actor.id = "1"
        actor.vehicle_type = "typhoon_h480"
        actor.uav_num = 6
        actor._model_names = None
        actor._actor_indices = {}
        actor._uav_indices = {}
        actor.actor_positions = {}
        actor.uav_positions = {}
        actor.gazebo_uav_pose = [Point() for _ in range(6)]
        actor.gazebo_uav_twist = [Point() for _ in range(6)]

        def states(names):
            return SimpleNamespace(
                name=names,
                pose=[Pose(position=Point(x=float(i))) for i in range(len(names))],
                twist=[Twist() for _ in names],
            )

        actor.actor_states_callback(states(["ground_plane", "actor_0", "actor_1",
                                            "typhoon_h480_0"]))
        self.assertEqual(actor._actor_state_pose.x, 2.0)
        actor.actor_states_callback(states(["ground_plane", "actor_1",
                                            "typhoon_h480_0"]))
        self.assertEqual(actor._actor_state_pose.x, 1.0)
        self.assertNotIn("actor_0", actor.actor_positions)

    def test_teleportation_wait_sleeps_without_printing(self):
        actor = ControlActor.__new__(ControlActor)
        actor.id = "0"
        actor.teleportation_interval = 25
        actor.create_human_point = lambda: (5, 6)
        clock = [0.0]
        sleep_count = [0]

        def sleep():
            clock[0] += 1.0
            sleep_count[0] += 1

        service = mock.Mock(return_value=SimpleNamespace(success=True))
        with mock.patch("control_actor.rospy.get_time", side_effect=lambda: clock[0]), \
             mock.patch("control_actor.rospy.is_shutdown", return_value=False), \
             mock.patch("control_actor.rospy.Rate", return_value=SimpleNamespace(sleep=sleep)), \
             mock.patch("control_actor.rospy.ServiceProxy", return_value=service), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            actor.actor_teleportation_callback(SimpleNamespace(data=0.0))

        self.assertEqual(sleep_count[0], 25)
        self.assertEqual(service.call_count, 1)
        self.assertEqual(output.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
