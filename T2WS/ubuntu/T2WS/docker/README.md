# T2WS on Ubuntu

This container runs ROS 2 Kilted beside a Booster Studio virtual T2 on native
Ubuntu. It joins the virtual robot's network namespace so both containers share
DDS loopback traffic. The repository is mounted at `/ws`, and GUI applications
use the host's native X11 socket.

## Start

1. Start Docker, Booster Studio, and the virtual T2.
2. In a host terminal, find the virtual-robot container:

       docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"

3. Grant local container root access to the current X11 display, then start
   T2WS:

       xhost +si:localuser:root
       export SIM_CONTAINER="<virtual-t2-container-name>"
       cd T2WS/ubuntu/T2WS/docker
       docker compose up -d --build
       docker compose exec t2ws bash

4. Verify ROS and X11 inside T2WS:

       ros2 topic list
       echo "$DISPLAY"
       rviz2

If Booster Studio recreates or renames the virtual-robot container, set
`SIM_CONTAINER` again and run:

    docker compose up -d --force-recreate

After stopping all T2WS GUI applications, revoke the temporary X11 permission:

    xhost -si:localuser:root

## Revo2 finger TF (`revo2_tf` service)

`docker compose up -d` also starts `revo2_tf` (container `t2ws-revo2-tf`). It publishes the Revo2
finger frames (`<side>_base_link`, finger links, `*_tip_link`, `*_touch_link`) under
`<side>_wrist_roll_link` in `/tf` and `/tf_static`, from the sim plugin's `/revo2/hand_state`.
See `t2_robot_side/src/robot/manipulation/revo2_tf/README.md`.

    docker compose ps                  # t2ws-dev and t2ws-revo2-tf running
    docker compose logs -f revo2_tf    # "Robot initialized" x2, "receiving hand state"
    docker compose up -d t2ws          # dev container only
    docker compose stop revo2_tf       # stop it (do this before running the launch by hand)

On the real robot T2WS only sees the robot-side relays (`/t2/...`); start it with
`REVO2_TF_TOPICS=t2` so it reads `/t2/revo2/hand_state` and publishes `/t2/tf`, `/t2/tf_static`
(default `sim` = Booster Studio).

Nothing in the Booster Studio container is modified. DDS uses UDP over the
shared network namespace according to `fastdds_profiles.xml`.
