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

## Revo2 finger TF (`revo2_tf`, inside `t2ws`)

The `t2ws` container starts `revo2_tf.launch.py` in the background (see `command` in `compose.yaml`).
It publishes the Revo2 finger frames (`<side>_base_link`, finger links, `*_tip_link`, `*_touch_link`,
grasp frames) under `<side>_wrist_roll_link` in `/t2/tf` and `/t2/tf_static`, from the finger joints
that the T2 agent appends to `/t2/joint_states`. Same in Booster Studio and on the real robot; the
agent must run. See `t2_robot_side/src/robot/manipulation/revo2_tf/README.md`.

    docker compose logs -f t2ws        # launch output: "Robot initialized" x2
    docker compose restart t2ws        # the launch is not restarted if it dies; this restarts it

Nothing in the Booster Studio container is modified. DDS uses UDP over the
shared network namespace according to `fastdds_profiles.xml`.
