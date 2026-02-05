FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    python3-numpy \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# Create ubuntu user with password auth for SSH
RUN useradd -m -s /bin/bash ubuntu && echo 'ubuntu:ubuntu' | chpasswd
RUN mkdir -p /run/sshd
# Allow password auth
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin no/' /etc/ssh/sshd_config
# Add ROS2 sourcing to ubuntu's bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/ubuntu/.bashrc

COPY nodes/ /home/ubuntu/nodes/
COPY scripts/ /home/ubuntu/scripts/
RUN chmod +x /home/ubuntu/scripts/*.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 22

ENTRYPOINT ["/entrypoint.sh"]
