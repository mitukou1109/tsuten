source /opt/ros/noetic/setup.bash

OVERLAY_SETUP_FILE=/root/catkin_ws/devel/setup.bash
if [[ -e $OVERLAY_SETUP_FILE ]]; then
  source $OVERLAY_SETUP_FILE
fi
