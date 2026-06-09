#!/bin/bash
CONTAINER="jetson-ros2-humble"

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    docker exec "$CONTAINER" pkill -SIGTERM -f "teleOp" 2>/dev/null
    sleep 2
    docker exec "$CONTAINER" pkill -9 -f "teleOp" 2>/dev/null
    docker exec "$CONTAINER" pkill -9 -f "ros2" 2>/dev/null
    echo "ROS processes killed in container: $CONTAINER"
else
    pkill -SIGTERM -f "teleOp" 2>/dev/null
    sleep 2
    pkill -9 -f "teleOp" 2>/dev/null
    pkill -9 -f "ros2" 2>/dev/null
    echo "ROS processes killed locally."
fi
