# Basis-Image
FROM ros:jazzy

# Installiere Git, rosdep
RUN apt-get update && rm -rf /var/lib/apt/lists/*

# Setze das Arbeitsverzeichnis
WORKDIR /ros2_ws/src

# Klone das ROS2-Node-Repository
RUN git clone https://github.com/wggRobotic/ros2-waveshare-servo-driver-node.git

# Installiere ROS-Abhängigkeiten
WORKDIR /ros2_ws
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y


# Setze die Shell auf bash
SHELL ["/bin/bash", "-c"]

# Baue den Workspace
RUN . /opt/ros/jazzy/setup.bash && colcon build --symlink-install

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run waveshare_servo_driver driver"]
