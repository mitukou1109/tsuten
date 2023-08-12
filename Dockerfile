FROM ros:noetic

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt-get install -y git python3-catkin-tools cmake pkg-config

WORKDIR /root

RUN git clone https://github.com/mitukou1109/tsuten_mainboard.git -b noetic
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git

WORKDIR /root/YDLidar-SDK/build

RUN cmake .. && make && make install

WORKDIR /root/catkin_ws/src

RUN git clone https://github.com/mitukou1109/tsuten.git -b noetic
RUN git clone https://github.com/mitukou1109/tsuten_real_robot.git -b noetic
RUN git clone https://github.com/mitukou1109/tsuten_simulation.git -b noetic
RUN git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git

RUN rosdep update && rosdep install -i -y --from-paths .

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

COPY setup_ros.bash ./
RUN echo "" >> /root/.bashrc
RUN cat setup_ros.bash >> /root/.bashrc

WORKDIR /root/catkin_ws