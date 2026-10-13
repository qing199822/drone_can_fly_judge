#!/usr/bin/env python3
import importlib.util
from pathlib import Path

import numpy as np

path = Path(__file__).parents[1] / "scripts" / "road_follow.py"
spec = importlib.util.spec_from_file_location("road_follow", path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

detector = object.__new__(mod.RoadDetector)
detector.last_heading = 0.25
detector.has_heading = True
mask = np.zeros((200, 300), dtype=np.uint8)
mask[30:180, 110:190] = 255
candidates = detector.build_candidates(mask, 0.0, 0.1, 0.8)
assert candidates and len(candidates[0]) == 4
assert abs(candidates[0][1]) < 0.1
assert candidates[0][2] > 0.0

shifted = np.zeros_like(mask)
shifted[30:180, 190:280] = 255
shifted_candidates = detector.build_candidates(shifted, 0.5, -0.2, 0.7)
assert shifted_candidates[0][1] > 0.2
assert shifted_candidates[0][0] == 0.0
assert detector.build_candidates(mask, 0.0, 0.0, 0.0) == []
assert mod.mission_stage("STAGE:ROAD") == "ROAD"
assert mod.mission_stage("STAGE:PATROL:seq") == "PATROL:seq"
assert mod.mission_stage("DISABLED") == ""
assert mod.mission_stage("STAGE:ROAD") == "ROAD"
assert mod.mission_stage("STAGE:PATROL") == "PATROL"
print("road mask geometry and stage parsing tests passed", candidates, shifted_candidates)
