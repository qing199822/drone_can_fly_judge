# Flight Safety Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the confirmed gas-station collision, enforce the 6 metre mission limit, and make launcher cleanup terminate reliably.

**Architecture:** Keep fixes at their sources: static clearance in the mission, per-aircraft completion in takeoff, and correct signals in the owning launcher. Use focused source/config tests followed by a full Gazebo/PX4 run.

**Tech Stack:** ROS Noetic, Gazebo 11, PX4 SITL 1.11, C++14, Python unittest, Bash.

---

### Task 1: Gas Station Clearance

**Files:**
- Modify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Modify: `src/mix_nav/task_manager/launch/mission_down.json`

- [ ] Add `gas_station_73` collision bounds to the static obstacle test.
- [ ] Run the test and confirm the existing drone 3 segments fail.
- [ ] Route drone 3 north of the expanded bounds and east before descending.
- [ ] Run the test and confirm it passes.

### Task 2: Six Metre Limit

**Files:**
- Modify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Modify: `src/mix_nav/fly/test/test_fly_launch.py`
- Modify: `src/mix_nav/fly/src/fly_takeoff.cpp`

- [ ] Add a strict `z < 6.0` assertion for every mission JSON waypoint.
- [ ] Add a failing regression test requiring climb publication to skip completed aircraft.
- [ ] Publish climb commands only to aircraft whose mission flag is false.
- [ ] Run both focused test modules.

### Task 3: Reliable Cleanup

**Files:**
- Modify: `tests/test_one_click_launch.py`
- Modify: `1.sh`

- [ ] Add a failing test requiring `SIGTERM` for background process groups and the simulator.
- [ ] Replace the cleanup `SIGINT` signals with `SIGTERM`.
- [ ] Run the launcher tests and Bash syntax check.

### Task 4: Integration And Publication

**Files:**
- Modify: `docs/TROUBLESHOOTING.md`
- Modify: `README.md`

- [ ] Build and run all project-owned tests.
- [ ] Run six aircraft with synchronized pose and contact monitoring.
- [ ] Confirm every maximum altitude is below 6 metres and drone 3 never contacts the gas station.
- [ ] Update the debugging status and evidence.
- [ ] Stage all task files except `.vscode/settings.json`, commit, and push `main` to `public`.
