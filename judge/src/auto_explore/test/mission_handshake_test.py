#!/usr/bin/env python3
from pathlib import Path

area = Path(__file__).resolve().parents[1] / "src" / "area_mission_node.cpp"
tracking = Path(__file__).resolve().parents[2] / "tracking" / "src" / "state_machine.cpp"
a = area.read_text()
t = tracking.read_text()
for token in ["PAUSE:", "RESUME:", 'PAUSED:" + pause_seq_', 'RESUMING:" + resume_seq_', 'RESUMED:" + resume_seq_']:
    assert token in a, token
for token in ['pause_cmd.data = "PAUSE:" + seq', 'waitForMissionAck("PAUSED:" + seq', 'resume_cmd.data = "RESUME:"']:
    assert token in t, token
print("mission handshake protocol checks passed")
