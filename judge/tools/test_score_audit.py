#!/usr/bin/env python3
"""Check that referee audit records explain detection resets and deletion."""

import contextlib
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

SCORE_CAL_PATH = Path(__file__).resolve().parents[1] / "score_cal.py"
spec = importlib.util.spec_from_file_location("workspace_score_cal", SCORE_CAL_PATH)
score_cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(score_cal)

score_cal.SCORE_OUTPUT_PATH = "/dev/null"


class DetectionAuditTest(unittest.TestCase):
    def setUp(self):
        self.clock = [100.0]
        score_cal.left_actors = list(range(score_cal.actor_num))
        score_cal.find_actors = list(range(score_cal.actor_num))
        score_cal.actors_pos = [None] * score_cal.actor_num
        score_cal.actors_pos[0] = SimpleNamespace(x=10.0, y=20.0)
        score_cal.count_flag = [False] * score_cal.actor_num
        score_cal.topic_arrive_time = [0.0] * score_cal.actor_num
        score_cal.find_time = [0.0] * score_cal.actor_num
        score_cal.detection_report_count = [0] * score_cal.actor_num
        score_cal.detection_max_gap = [0.0] * score_cal.actor_num
        score_cal.detection_max_error = [0.0] * score_cal.actor_num
        score_cal.find_actor_pub = [mock.Mock() for _ in range(score_cal.actor_num)]
        score_cal.find_finish = 0
        score_cal.target_finish = 0
        score_cal.mission_finished = False
        score_cal.mission_completed = False
        score_cal.start_time = 100.0
        self.msg = SimpleNamespace(cls="green", x=10.3, y=20.0,
                                   _connection_header={"topic": "/actor_green_info"})
        self.get_time = mock.patch.object(score_cal.rospy, "get_time",
                                          side_effect=lambda: self.clock[0])
        self.publish_score = mock.patch.object(score_cal, "_publish_score")
        self.delete_model = mock.patch.object(
            score_cal, "del_model", create=True,
            return_value=SimpleNamespace(success=True))
        self.get_time.start()
        self.publish_score.start()
        self.delete_model.start()
        self.addCleanup(self.get_time.stop)
        self.addCleanup(self.publish_score.stop)
        self.addCleanup(self.delete_model.stop)

    def test_success_log_has_full_15_second_basis(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            for index in range(31):
                self.clock[0] = 100.0 + index * 0.5
                score_cal._process_actor_detection(self.msg, [0])

        log = output.getvalue()
        self.assertEqual(log.count("event=report"), 31)
        self.assertIn("decision=accepted reason=timer_started", log)
        self.assertIn("result=success start=100.000000 end=115.000000 duration=15.000 reports=31", log)
        self.assertIn("max_gap=0.500 max_error=0.300", log)
        self.assertNotIn(0, score_cal.left_actors)

    def test_gap_and_error_log_reset_reasons(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            score_cal._process_actor_detection(self.msg, [0])
            self.clock[0] = 102.0
            score_cal._process_actor_detection(self.msg, [0])
            self.clock[0] = 102.5
            self.msg.x = 12.0
            score_cal._process_actor_detection(self.msg, [0])

        log = output.getvalue()
        self.assertIn("gap=2.000", log)
        self.assertIn("decision=reset reason=report_gap", log)
        self.assertIn("decision=reset reason=coordinate_error", log)
        self.assertFalse(score_cal.count_flag[0])


if __name__ == "__main__":
    unittest.main()
