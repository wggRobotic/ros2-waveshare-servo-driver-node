# Basis-Image
FROM ros:jazzy

# Installiere Git, rosdep
RUN apt-get update && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src
COPY ./waveshare_servo_driver ./waveshare_servo_driver


# Installiere ROS-Abhängigkeiten
WORKDIR /ros2_ws
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y


# Setze die Shell auf bash
SHELL ["/bin/bash", "-c"]

# Baue den Workspace
RUN . /opt/ros/jazzy/setup.bash && colcon build --symlink-install

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run waveshare_servo_driver driver"]
