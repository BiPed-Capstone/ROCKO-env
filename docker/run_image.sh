#!/usr/bin/env bash
#
# Run the ROCKO-env Docker image in the shell. Use this to run commands inside the container.
# For example, use this to build the ROS packages or run the simulator.

echo "Running the container for macOS..."
docker build -t rocko-env-dev .
docker compose -f compose.yml -f compose.mac.yml up -d
docker exec -it rocko-env-dev /bin/bash
