# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set up the environment to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    ca-certificates \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libx11-dev \
    libxrender-dev \
    libxrandr-dev \
    build-essential \
    udev \
    usbutils \
    libusb-1.0-0-dev \
    xserver-xorg \
    x11-xserver-utils \
    x11-utils \
    mesa-utils \
    pkg-config \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set up NVIDIA drivers
RUN apt-get update && apt-get install -y \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*

# Create required directories for NVIDIA
RUN mkdir -p /usr/local/share/glvnd/egl_vendor.d/
RUN echo '{"file_format_version" : "1.0.0", "ICD" : {"library_path" : "libEGL_nvidia.so.0"}}' > /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install ROS Noetic
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Install Python tools and additional ROS packages
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rviz \
    ros-noetic-ros-controllers \
    ros-noetic-rqt\
    ros-noetic-rqt-common-plugins\
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Gazebo 11
RUN apt-get update && apt-get install -y \
    gazebo11 \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages for OpenMANIPULATOR-X (without joystick-drivers)
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-industrial-core \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench \
    ros-noetic-robotis-manipulator \
    ros-noetic-pcl-ros \
    ros-noetic-joy \
    ros-noetic-control-toolbox \
    ros-noetic-controller-interface \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    libboost-dev \
    libeigen3-dev \
    libtinyxml-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies for USB device access
RUN apt-get update && apt-get install -y \
    usb-modeswitch \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone the core dependencies first
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git 
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench.git 
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git 

# Build the core dependencies first
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make --only-pkg-with-deps dynamixel_sdk dynamixel_workbench_msgs robotis_manipulator"

WORKDIR /root/catkin_ws/src/DynamixelSDK/python/
RUN python3 setup.py install
# Install any missing ROS dependencies
WORKDIR /root/catkin_ws
RUN apt-get update && rosdep install --from-paths src --ignore-src -y || true

# Add user for accessing USB devices
RUN groupadd -r docker && usermod -aG docker root

COPY ./pr_robotics ./src/pr_robotics

# Try building everything except problematic packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /root/catkin_ws/devel/setup.bash && \
    catkin_make"


# (Your previous ROS and system installations are here)

# Install Miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    /opt/conda/bin/conda clean -tipy

# Add conda to the PATH
ENV PATH /opt/conda/bin:$PATH

# Initialize conda and install packages in the base environment
SHELL ["/bin/bash", "-c"]

# 2. Now run the initialization and installation commands.
#    Since the shell is bash, it will correctly process all conda scripts.
# Initialize Conda and install ALL Python dependencies in one go.
# This is more reliable than using pip in a separate step.
RUN conda init bash && \
    source /root/.bashrc && \
    echo "==> Installing Conda packages: pinocchio, pyyaml, pyserial, rospkg..." && \
    conda install -c conda-forge pinocchio pyyaml pyserial rospkg matplotlib ezdxf empy --yes && \
    echo "==> Cleaning up conda packages..." && \
    conda clean -afy


ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0
RUN apt-get update && apt-get install -y x11-apps && rm -rf /var/lib/apt/lists/*

# Source the ROS environment by default
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set the default command
CMD ["bash"]