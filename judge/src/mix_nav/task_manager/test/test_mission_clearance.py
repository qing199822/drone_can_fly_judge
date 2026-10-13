#!/usr/bin/env python3

import json
import unittest
from pathlib import Path


MISSION_FILE = Path(__file__).resolve().parents[1] / "launch" / "mission_down.json"
MISSION_FILES = sorted(MISSION_FILE.parent.glob("mission_*.json"))
HORIZONTAL_CLEARANCE = 2.0
MAX_FLIGHT_ALTITUDE = 6.0

# Bounds are calculated from the collision meshes and transforms in robocup.world.
STATIC_OBSTACLES = (
    ("gas_station_73", 51.59, 72.17, -36.63, -6.64, 8.98),
    ("fast_food_93", 14.12, 39.29, -22.22, -5.64, 11.02),
    ("house_2_125", -5.96, 6.77, -19.82, -10.03, 7.20),
    ("house_3_156", -35.78, -30.98, -25.68, -13.03, 10.62),
    ("house_3_157", -36.40, -23.75, -35.32, -30.52, 10.62),
    ("house_3_158", -28.15, -23.34, -27.65, -15.00, 10.62),
    ("lamp_post_189", -35.70, -34.50, -4.30, -3.10, 7.20),
    ("lamp_post_190", -25.80, -24.60, 3.10, 4.30, 7.20),
    ("lamp_post_191", -3.60, -2.40, -4.30, -3.10, 7.20),
    ("lamp_post_192", 8.40, 9.60, 3.10, 4.30, 7.20),
    ("lamp_post_193", 20.40, 21.60, -4.30, -3.10, 7.20),
    ("lamp_post_194", 26.40, 27.60, 3.10, 4.30, 7.20),
    ("lamp_post_195", 57.40, 58.60, -4.30, -3.10, 7.20),
    ("lamp_post_196", 70.40, 71.60, 3.10, 4.30, 7.20),
    ("lamp_post_197", 83.40, 84.60, -4.30, -3.10, 7.20),
    ("lamp_post_198", 89.90, 91.10, 3.10, 4.30, 7.20),
)


def segment_intersects_box(start, end, x_min, x_max, y_min, y_max):
    """Return whether a 2-D segment intersects an axis-aligned box."""
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    t_min = 0.0
    t_max = 1.0

    for origin, delta, lower, upper in (
        (start["x"], dx, x_min, x_max),
        (start["y"], dy, y_min, y_max),
    ):
        if abs(delta) < 1e-9:
            if origin < lower or origin > upper:
                return False
            continue

        entry = (lower - origin) / delta
        exit_ = (upper - origin) / delta
        if entry > exit_:
            entry, exit_ = exit_, entry
        t_min = max(t_min, entry)
        t_max = min(t_max, exit_)
        if t_min > t_max:
            return False

    return True


class MissionClearanceTest(unittest.TestCase):
    def test_all_mission_waypoints_are_strictly_below_six_metres(self):
        violations = []
        for mission_file in MISSION_FILES:
            with mission_file.open(encoding="utf-8") as mission_stream:
                missions = json.load(mission_stream)
            for mission in missions:
                for index, waypoint in enumerate(mission["waypoints"], start=1):
                    if waypoint["z"] >= MAX_FLIGHT_ALTITUDE:
                        violations.append(
                            f"{mission_file.name}: {mission['vehicle_id']} waypoint "
                            f"{index} has z={waypoint['z']}"
                        )

        self.assertEqual([], violations, "\n" + "\n".join(violations))

    def test_flight_segments_clear_known_static_buildings(self):
        with MISSION_FILE.open(encoding="utf-8") as mission_stream:
            missions = json.load(mission_stream)

        collisions = []
        for mission in missions:
            vehicle_id = mission["vehicle_id"]
            waypoints = mission["waypoints"]
            for index, (start, end) in enumerate(zip(waypoints, waypoints[1:]), start=1):
                segment_floor = min(start["z"], end["z"])
                for name, x_min, x_max, y_min, y_max, z_max in STATIC_OBSTACLES:
                    if segment_floor > z_max:
                        continue
                    if segment_intersects_box(
                        start,
                        end,
                        x_min - HORIZONTAL_CLEARANCE,
                        x_max + HORIZONTAL_CLEARANCE,
                        y_min - HORIZONTAL_CLEARANCE,
                        y_max + HORIZONTAL_CLEARANCE,
                    ):
                        collisions.append(
                            f"{vehicle_id} segment {index}->{index + 1} intersects {name}"
                        )

        self.assertEqual([], collisions, "\n" + "\n".join(collisions))


if __name__ == "__main__":
    unittest.main()
