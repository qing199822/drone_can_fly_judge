# Public Debug Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce, verify, commit, and publish a reproducible public debugging release of the six-drone RoboCup simulation.

**Architecture:** Keep the Catkin workspace self-contained for project code while treating PX4, XTDrone, Gazebo models, and Python environments as documented sibling dependencies. Use focused project tests plus a monitored six-drone integration run as the release gate.

**Tech Stack:** Ubuntu 20.04, ROS Noetic, Gazebo 11, PX4 v1.11.0-beta1, Catkin/CMake, Python 3.8, C++14, PyTorch/Ultralytics.

---

### Task 1: Recoverable Backup

**Files:**
- Create: `../backups/2025_ZZU_FLY-20260725-232453.tar.gz`
- Create: `../backups/2025_ZZU_FLY-20260725-232453.patch`
- Create: `../backups/2025_ZZU_FLY-20260725-232453.bundle`

- [x] Create a source snapshot excluding build output and caches.
- [x] Export the binary Git diff and complete Git history bundle.
- [x] Verify the archive listing, checksums, and bundle integrity.

### Task 2: Public Repository Documentation

**Files:**
- Create: `.gitignore`
- Modify: `README.md`
- Create: `docs/ENVIRONMENT.md`
- Create: `docs/TROUBLESHOOTING.md`
- Create: `CONTRIBUTING.md`
- Create: `requirements-yolo.txt`

- [x] Exclude Catkin output, logs, caches, and local workspace metadata.
- [x] Record the verified environment and external directory contract.
- [x] Document launch, validation, camera rendering, cleanup, and issue-reporting steps.

### Task 3: Build and Automated Tests

**Files:**
- Verify: `src/mix_nav/task_manager/test/test_mission_clearance.py`
- Verify: `tests/test_one_click_launch.py`
- Verify: `tests/test_graphics_environment.py`
- Verify: `src/mix_nav/simple_navigator/test/velocity_continuity.test`
- Verify: `src/pose_init/test/pose_namespace.test`

- [ ] Run a fresh Catkin build and require exit code zero.
- [ ] Run all project-owned Python tests and require zero failures.
- [ ] Run navigation and pose rostests and require zero failures.

### Task 4: Six-Drone Integration Run

**Files:**
- Verify: `1.sh`
- Verify: `src/mix_nav/task_manager/launch/mission_down.json`

- [ ] Start exactly one six-drone simulation with the corrected mission.
- [ ] Monitor connectivity, flight state, commands, poses, and all RGB/depth topics.
- [ ] Confirm no aircraft falls below the crash threshold or exceeds 6 metres.
- [ ] Stop the launcher and confirm no ROS/PX4/Gazebo/YOLO helper remains.

### Task 5: Publication Audit and Commit

**Files:**
- Stage only repository source, tests, documentation, required models, and weights.

- [ ] Scan staged content for secrets and GitHub file-size violations.
- [ ] Review the staged diff and generated file exclusions.
- [ ] Commit the verified release on a dedicated local branch.

### Task 6: GitHub Publication

**Files:**
- Update Git remote configuration only if the authenticated account cannot write the current origin.

- [ ] Detect an existing GitHub SSH or credential-helper identity without printing secrets.
- [ ] Push the release branch to a public GitHub repository.
- [ ] Verify the remote branch and repository visibility from GitHub.
