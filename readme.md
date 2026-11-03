## kitchen challenge - Booster T2

Simulation development environments:

- `T2WS/windows/T2WS`: ROS 2 Kilted sidecar for Booster Studio on Windows Docker Desktop.
- `T2WS/ubuntu/T2WS`: ROS 2 Kilted sidecar for Booster Studio on native Ubuntu with X11.

Both variants mount this repository at `/ws` and join the active virtual T2's
network namespace. See each variant's `docker/README.md` for launch instructions.
