FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install ROS dependencies and system libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-octomap \
    ros-noetic-octomap-mapping \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-octomap-server \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    ros-noetic-vision-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-tf2-ros \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-pcl-ros \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libprotobuf-dev \
    protobuf-compiler \
    qtbase5-dev \
    python3-catkin-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create Eigen symlink (needed by some packages)
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

# Setup catkin workspace
RUN mkdir -p /root/catkin_ws/src
COPY . /root/catkin_ws/src/Intent-MPC

# Build
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j$(nproc)"

# Setup environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/src/Intent-MPC/uav_simulator/gazeboSetup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
