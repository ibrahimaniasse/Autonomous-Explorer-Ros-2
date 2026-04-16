# --------- BASE STAGE ---------
FROM osrf/ros:humble-desktop AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# --------- BUILDER STAGE ---------
FROM base AS builder

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ./src /ros2_ws/src

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# --------- RUNTIME STAGE ---------
FROM base AS runtime

WORKDIR /ros2_ws

# Copy ONLY the install space from builder
COPY --from=builder /ros2_ws/install /ros2_ws/install
# Create empty src directory for potential bind mounts
RUN mkdir -p /ros2_ws/src

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
