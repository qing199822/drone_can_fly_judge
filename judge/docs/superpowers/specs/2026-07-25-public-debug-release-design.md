# Public Debug Release Design

## Goal

Publish the current six-drone RoboCup simulation as a reproducible public debugging repository without uploading machine-generated output, local credentials, or the external 1.2GB PX4 environment.

## Repository Boundary

The repository contains the Catkin source workspace, project launchers, models, mission files, YOLO weights, project tests, and compatibility changes to the bundled Gazebo ROS packages. PX4, XTDrone, Gazebo model collections, Python virtual environments, build output, ROS logs, and competition documents remain external.

## Documentation

The root README is the entry point. `docs/ENVIRONMENT.md` records the exact verified version matrix, sibling directory layout, build sequence, PX4 archive constraint, and YOLO environment. `docs/TROUBLESHOOTING.md` records camera rendering, MAVROS, OFFBOARD, collision, and cleanup diagnostics. `CONTRIBUTING.md` defines the minimum reproduction and verification evidence expected from collaborators.

## Verification

Static project tests and a clean Catkin build must pass. A six-drone run must then demonstrate MAVROS connectivity, airborne flight, command publication, RGB/depth messages for all six vehicles, and no collision on the corrected mission route. All processes must be stopped after verification.

## Publication

Before committing, scan tracked content for credentials and files above GitHub limits. Commit only source, documentation, tests, models, and required weights. Push through an already configured GitHub identity; installing or changing system-level authentication tooling requires separate confirmation.

