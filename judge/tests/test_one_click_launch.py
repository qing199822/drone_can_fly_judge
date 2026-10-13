#!/usr/bin/env python3

import pathlib
import unittest


class OneClickLaunchTest(unittest.TestCase):
    def test_cleanup_uses_terminable_signal_for_background_jobs(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text()

        self.assertIn('MISSION_PID=""', script)
        self.assertIn('kill -TERM -- "-$pid"', script)
        self.assertIn('kill -TERM "$MISSION_PID"', script)
        self.assertIn('kill -TERM "$SIMULATION_PID"', script)
        self.assertNotIn('kill -INT -- "-$pid"', script)
        self.assertNotIn('kill -INT "$SIMULATION_PID"', script)

    def test_mission_launch_is_owned_by_cleanup(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text()

        mission_launch = (
            'roslaunch look_up down_resume.launch num_drones:="$NUM_DRONES" '
            'mission_filename:="$MISSION_FILE" &\n'
            'MISSION_PID=$!\n'
            'wait "$MISSION_PID"'
        )
        self.assertIn(mission_launch, script)

    def test_simulator_ready_timeout_can_be_configured(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text()

        self.assertIn(
            'READY_TIMEOUT_SECONDS="${READY_TIMEOUT_SECONDS:-180}"',
            script,
            "slow six-vehicle startup must allow a longer timeout via the environment",
        )

    def test_communication_bridge_starts_before_mission(self):
        script = (pathlib.Path(__file__).parents[1] / "1.sh").read_text()

        bridge = 'start_communication "$XTDRONE_PYTHON"'
        mission = 'roslaunch look_up down_resume.launch'
        self.assertIn(bridge, script)
        self.assertIn(mission, script)
        self.assertLess(
            script.index(bridge),
            script.index(mission),
            "XTDrone communication bridges must start before the mission sends commands",
        )


if __name__ == "__main__":
    unittest.main()
