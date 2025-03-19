# Basis: ROS2 Base Image (z.B. basierend auf ros:jazzy)
FROM ros2-base-image

# Installiere Git
RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# Setze das Arbeitsverzeichnis im Container
WORKDIR /ros2_ws/src

# Klone das spezifische ROS2 Node Repository
RUN git clone https://github.com/wggRobotic/ros2-waveshare-servo-driver-node.git

# Wechsel in das übergeordnete Verzeichnis und baue den ROS2 Workspace
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# Setze die Shell auf bash, um die Setup-Skripte korrekt zu sourcen
SHELL ["/bin/bash", "-c"]

# Sourcen der ROS2-Umgebung und Start des gewünschten Nodes
# TODO: Ersetze <PACKAGE_NAME> und <NODE_EXECUTABLE> durch die entsprechenden Werte des ROS2-Pakets
CMD "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run <PACKAGE_NAME> <NODE_EXECUTABLE>"
