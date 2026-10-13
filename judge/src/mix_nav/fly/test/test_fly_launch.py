#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


class FlyLaunchTest(unittest.TestCase):
    def test_completed_takeoff_does_not_shutdown_mission(self):
        launch_file = pathlib.Path(__file__).parents[1] / "launch" / "fly.launch"
        root = ET.parse(str(launch_file)).getroot()
        takeoff_node = root.find("./node[@name='confident_takeoff_node']")

        self.assertIsNotNone(takeoff_node)
        self.assertNotEqual(
            takeoff_node.get("required", "false").lower(),
            "true",
            "a normally completed takeoff node must not stop the entire mission launch",
        )

    def test_completed_aircraft_do_not_receive_more_climb_commands(self):
        source_file = pathlib.Path(__file__).parents[1] / "src" / "fly_takeoff.cpp"
        compacted_source = "".join(source_file.read_text().split())

        self.assertIn(
            "if(!mission_done_flags_[i]){vel_pubs_[i].publish(climb_twist);}",
            compacted_source,
            "each aircraft must stop climbing as soon as it reaches target altitude",
        )


if __name__ == "__main__":
    unittest.main()
