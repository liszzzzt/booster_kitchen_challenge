# T2WS on Windows

This container runs ROS 2 Kilted beside a Booster Studio virtual T2 on Windows
Docker Desktop. It joins the virtual robot's network namespace so both
containers share DDS loopback traffic. The repository is mounted at `/ws`.

## Start

1. Start Docker Desktop, Booster Studio, and the virtual T2.
2. Start VcXsrv on display `0` with TCP listening and access control disabled.
3. In PowerShell, find the virtual-robot container and start T2WS:

       docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"
       $env:SIM_CONTAINER="<virtual-t2-container-name>"
       cd T2WS\windows\T2WS\docker
       docker compose up -d --build
       docker compose exec t2ws bash

4. Verify ROS and X11 inside T2WS:

       ros2 topic list
       echo "$DISPLAY"
       rviz2

`DISPLAY` is fixed to `host.docker.internal:0.0` for VcXsrv. If Booster Studio
recreates or renames the virtual-robot container, set `SIM_CONTAINER` again and
run:

    docker compose up -d --force-recreate

Nothing in the Booster Studio container is modified. DDS uses UDP over the
shared network namespace according to `fastdds_profiles.xml`.
