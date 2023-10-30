#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice call /gazebo/reset_simulation "{}"
