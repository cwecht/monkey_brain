FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y gdb gcovr
RUN apt-get update && apt-get install -y python3-flask python3-waitress python3-expiringdict python3-rosdoc2
RUN apt-get update && apt-get install -y ros-jazzy-yasmin-demos
