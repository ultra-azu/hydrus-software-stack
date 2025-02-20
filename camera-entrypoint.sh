#!/bin/sh
set -e
. devel/setup.sh && \
	catkin_make -DCMAKE_BUILD_TYPE=Release -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/libcudart.so

trap "exit" SIGINT

while true; do
	echo "Do you want to display the ZED2i camera in RViz?"
	read -p "(y/n)" choice

	case $choice in 
		[Yy]* ) roslaunch zed_display_rviz display_zed2i.launch; break;;
		[Nn]* ) roslaunch --wait zed_wrapper zed2i.launch; break;;
		* ) echo "Invalid input. Please try again.";;
	esac
done