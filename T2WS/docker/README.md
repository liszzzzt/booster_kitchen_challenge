# Docker: run code against the Booster Studio sim container

    # start the sim from Booster Studio first
    cd docker
    docker compose up -d --build
    docker compose exec t2ws bash
    ros2 topic list          # should show the robot topics (domain 0)

The container joins the sim's network namespace (`network_mode: container:3qchq-t2`), because the
sim's DDS only listens on its own loopback. Nothing in the sim container is modified. Set
`SIM_CONTAINER=<name>` if the sim container has a different name, and recreate this container
whenever the sim is recreated. The repo is mounted at `/ws`. DDS is UDP-only
(see `fastdds_profiles.xml`).
