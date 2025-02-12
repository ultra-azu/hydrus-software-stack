#!/bin/sh
set -e
. devel/setup.sh && \
	catkin_make -DCMAKE_BUILD_TYPE=Release -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/libcudart.so
roslaunch zed_display_rviz display_zed2i.launch   