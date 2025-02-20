#!/bin/sh
set -e
. devel/setup.sh && \
	catkin_make -DCMAKE_BUILD_TYPE=Release -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/libcudart.

case $ZED_OPTION in 
  "noviz" ) roslaunch --wait zed_wrapper zed2i.launch ;;
  "rviz" ) roslaunch zed_display_rviz display_zed2i.launch ;;
esac   