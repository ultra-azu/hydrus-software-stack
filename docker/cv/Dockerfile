FROM ros:humble

WORKDIR /hydrus_ws/src/
RUN apt-get update && apt-get install -y \
python3-pip \
&& rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-python ultralytics

WORKDIR /hydrus_ws
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && colcon build'

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /hydrus_ws/install/setup.bash" >> ~/.bashrc

COPY ./hydrus-software-stack .
# ENTRYPOINT ["/bin/bash", "-c"]
# CMD ["source ~/.bashrc && ros2 run hydrus_cv camera"]