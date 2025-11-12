# Start from the official ROS Melodic image (Ubuntu 18.04, Python 3.6)
FROM osrf/ros:melodic-desktop-full

# Set up environment variables
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV PYTHON_EXECUTABLE=/usr/bin/python3
ENV PIP=pip3

# Ensure ROS repository is set up
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update && apt-get clean

# Install core dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-yaml \
    python3-defusedxml \
    python3-netifaces \
    zlib1g-dev \
    mesa-utils \
    cmake \
    build-essential \
    unzip \
    wget \
    python-qt4 \
    python-qt4-gl \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python 3 ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-rospkg \
    python3-catkin-pkg \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-melodic-xacro \
    ros-melodic-rviz \
    ros-melodic-joint-state-controller \
    ros-melodic-diff-drive-controller \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-control \
    ros-melodic-ros-base \
    ros-melodic-robot-localization \
    ros-melodic-mavros-msgs \
    ros-melodic-gmapping \
    ros-melodic-map-server \
    ros-melodic-amcl \
    ros-melodic-velocity-controllers \
    ros-melodic-twist-mux \
    ros-melodic-robot-state-publisher \
    ros-melodic-imu-tools \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-gazebo-plugins \
    ros-melodic-ros-control \
    ros-melodic-navigation \
    ros-melodic-move-base \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins \
    ros-melodic-base-local-planner \
    ros-melodic-clear-costmap-recovery \
    ros-melodic-costmap-2d \
    ros-melodic-nav-core \
    ros-melodic-navfn \
    ros-melodic-rotate-recovery \
    ros-melodic-carrot-planner \
    ros-melodic-dwa-local-planner \
    ros-melodic-fake-localization \
    ros-melodic-global-planner \
    ros-melodic-move-slow-and-clear \
    ros-melodic-imu-complementary-filter \
    ros-melodic-imu-filter-madgwick \
    ros-melodic-eigen-conversions \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-teb-local-planner \
    ros-melodic-realsense2-camera \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Update setuptools to avoid pkg_resources issues
RUN pip3 install --no-cache-dir setuptools==59.6.0

# Install catkin-pkg explicitly to ensure version 0.5.2
RUN pip3 install --no-cache-dir catkin-pkg==0.5.2

# Install Python packages via pip
RUN pip3 install --no-cache-dir \
    matplotlib \
    torch==1.7.1 \
    numpy==1.19.5 \
    gym==0.17.3 \
    tensorflow==1.14.0 \
    keras==2.2.4 \
    defusedxml \
    netifaces \
    protobuf==3.19.6 \
    pandas 

# Install the C++ API for PyTorch (LibTorch)
RUN wget https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-1.10.0%2Bcpu.zip && \
    unzip libtorch-shared-with-deps-1.10.0+cpu.zip -d /opt/ && \
    rm libtorch-shared-with-deps-1.10.0+cpu.zip

# Set up Python alternatives (use Python 2 for catkin tools, Python 3 for user scripts)
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 2
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1

RUN apt-get update && apt-get install -y \
    nano \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    kconfiglib \
    jsonschema \
    jinja2 \
    empy==3.3.4 \
    lxml 

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Install gazebo-realsense plugin
WORKDIR /opt
RUN git clone https://github.com/intel/gazebo-realsense.git && \
    cd gazebo-realsense && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

ENV GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/

# Install GeographicLib datasets required by MAVROS
RUN apt-get update && apt-get install -y wget \
 && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geographiclib_datasets.sh \
 && bash /tmp/install_geographiclib_datasets.sh \
 && rm /tmp/install_geographiclib_datasets.sh

RUN apt-get update && apt-get install -y \
    python-wstool \
    python-rosdep \
    ninja-build \
    stow \
    ros-melodic-pcl-conversions \
    ros-melodic-tf2-eigen \
    ros-melodic-eigen-conversions \
    ros-melodic-slam-toolbox \
    ros-melodic-multirobot-map-merge \
    ros-melodic-explore-lite \
    ros-melodic-velodyne-gazebo-plugins \
    ros-melodic-pointcloud-to-laserscan \
    libceres-dev \
    liblua5.3-dev \
    python-scipy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Add PX4 and Gazebo environment setup to bashrc
RUN echo "\
# PX4 Gazebo setup\n\
source /home/ros_workspace/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /home/ros_workspace/PX4-Autopilot /home/ros_workspace/PX4-Autopilot/build/px4_sitl_default\n\
\n\
# ROS package path for PX4\n\
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/ros_workspace/PX4-Autopilot\n\
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/ros_workspace/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic\n\
\n\
# Gazebo plugin path\n\
export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins\n\
" >> /root/.bashrc


# Set working directory
WORKDIR /home/ros_workspace

# Source ROS environment on each interactive shell


RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /home/cartographer_ws/install_isolated/setup.bash ]; then" >> /root/.bashrc && \
    echo "  source /home/cartographer_ws/install_isolated/setup.bash" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc
    
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Set default command to bash with ROS sourced
CMD ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash && source /home/ros_workspace/devel/setup.bash 2>/dev/null || true && echo 'Workspace is mounted at /home/ros_workspace' && bash"]