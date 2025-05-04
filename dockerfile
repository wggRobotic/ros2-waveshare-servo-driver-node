FROM ros:jazzy

# Installiere Abhängigkeiten
RUN apt-get update && apt-get install -y \
    git \
    python3-serial \
 && rm -rf /var/lib/apt/lists/*

# Arbeitsverzeichnis setzen
WORKDIR /ros2_ws/src

# Repo klonen
RUN git clone https://github.com/wggRobotic/ros2-waveshare-servo-driver-node.git

# ROS-Abhängigkeiten installieren
WORKDIR /ros2_ws
RUN rosdep update && rosdep install --from-paths src/ros2-waveshare-servo-driver-node --ignore-src -r -y

# Setze Shell auf bash
SHELL ["/bin/bash", "-c"]

# Build Workspace
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# Starte Node
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run waveshare_servo_driver driver"]
