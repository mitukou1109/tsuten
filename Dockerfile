FROM ros:humble

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt-get install -y git

WORKDIR /root

RUN git clone https://github.com/mitukou1109/tsuten_mainboard.git -b humble

WORKDIR /root/colcon_ws/src

RUN git clone https://github.com/mitukou1109/tsuten.git -b humble
RUN git clone https://github.com/mitukou1109/tsuten_real_robot.git -b humble
RUN git clone https://github.com/mitukou1109/tsuten_simulation.git -b humble
RUN git clone https://github.com/Kotakku/jsk_visualization.git

RUN rosdep install -y -i --from-paths .

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /root/colcon_ws