FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    python3-numpy \
    python3-psutil \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# SSH setup
RUN useradd -m -s /bin/bash ubuntu && echo 'ubuntu:ubuntu' | chpasswd
RUN mkdir -p /run/sshd
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin no/' /etc/ssh/sshd_config

# Copy colcon workspace source
COPY ros2_ws/src /home/ubuntu/ros2_ws/src

# Build the workspace
WORKDIR /home/ubuntu/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source ROS2 and workspace in bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/ubuntu/.bashrc && \
    echo 'source /home/ubuntu/ros2_ws/install/setup.bash' >> /home/ubuntu/.bashrc

# Diagnostic scripts
COPY scripts/ /home/ubuntu/scripts/
RUN chmod +x /home/ubuntu/scripts/*.sh

# Pre-populated logs with realistic operational history
COPY logs/ /home/ubuntu/logs/
RUN chown -R ubuntu:ubuntu /home/ubuntu/logs

# Reference documentation
COPY docs/ /home/ubuntu/docs/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN chown -R ubuntu:ubuntu /home/ubuntu

EXPOSE 22

ENTRYPOINT ["/entrypoint.sh"]
