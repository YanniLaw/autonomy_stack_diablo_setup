#!/bin/bash

# Source ROS 2 and your workspace (modify as needed)
source /opt/ros/humble/setup.bash
source /home/temi/autonomy_stack_diablo_setup/install/setup.bash
# start web server
bash /home/temi/diablo_web_server/scripts/start_web_control.sh
