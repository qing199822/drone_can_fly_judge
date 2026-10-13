# Flight Safety Fixes Design

## Goal

Prevent the confirmed drone 3 collision with `gas_station_73`, keep every aircraft below the 6 metre competition limit, and make simulation shutdown reliable.

## Evidence

- Gazebo reported `gas_station_73::link::collision` against `typhoon_h480_3::base_link::base_link_collision` at `x=53.20, y=-6.96, z=3.87`.
- The gas-station collision mesh occupies `x=51.59..72.17`, `y=-36.63..-6.64`, and reaches about 8.98 metres, so climbing within the limit cannot clear it.
- The takeoff node kept publishing `+0.8 m/s` to aircraft that had already reached 3 metres while waiting for the slowest aircraft. This produced observed peaks of 7.62, 8.01, and 9.09 metres.
- Background Bash jobs ignored `SIGINT`, leaving the launcher's cleanup waiting indefinitely.

## Design

Route drone 3 around the north side of the gas station with the existing 2 metre horizontal clearance policy. Keep the altitude at 3.5 metres and move the descent toward the southern lane until after the aircraft has passed the station's east edge.

In the takeoff loop, publish climb velocity only for aircraft whose completion flag is false. Aircraft that reached the target are handed back to the existing zero navigator command and XTDrone hover behavior.

Use `SIGTERM` for launcher-owned background process groups and the simulator process. The existing traps and waits remain responsible for orderly cleanup.

## Verification

- A mission-clearance test must fail on the original gas-station crossing and pass on the detour.
- A takeoff source regression test must fail while climb commands are sent to completed aircraft.
- A launcher test must require termination-safe cleanup signals.
- Every mission JSON waypoint must have `z < 6.0`.
- A six-aircraft integration run must show no gas-station contact, no loss of flight, and maximum Gazebo altitude below 6 metres.
