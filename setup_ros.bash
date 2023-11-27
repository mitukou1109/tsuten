source /opt/ros/humble/setup.bash

OVERLAY_SETUP_FILE=/root/colcon_ws/install/setup.bash
if [[ -e $OVERLAY_SETUP_FILE ]]; then
  source $OVERLAY_SETUP_FILE
fi
