#!/bin/bash

# This will launch the nodes and disown
# the processes so they can continue running
# after the terminal window has been closed

ros2 launch rover_launch.py &
disown
