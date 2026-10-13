#!/usr/bin/env python3

import pathlib
import unittest


STATE_MACHINE_SOURCE = (
    pathlib.Path(__file__).parents[1] / "src" / "tracking" / "src" / "state_machine.cpp"
)


class TrackingMuxWiringTest(unittest.TestCase):
    def test_tracking_commands_publish_to_external_mux_input(self):
        source = STATE_MACHINE_SOURCE.read_text()
        compacted_source = "".join(source.split())
        publisher_line = next(
            line for line in source.splitlines() if "cmd_vel_pub_ = nh_.advertise" in line
        )

        self.assertIn("external_command_topic", publisher_line)
        self.assertIn(
            'external_command_topic="/"+vehicle_type_+"_"+vehicle_id_+'
            '"/mux_inputs/external/pose_cmd"',
            compacted_source,
        )
        self.assertNotIn('cmd_vel_pub_=nh_.advertise<geometry_msgs::Twist>(xtdrone_namespace', compacted_source)


if __name__ == "__main__":
    unittest.main()
