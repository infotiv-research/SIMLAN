

FROM althack/ros2:humble-cuda-gazebo-nvidia

# future work: ROS_DISTRO -> iron

ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV DEBIAN_FRONTEND=noninteractive

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# To fix the expired key issue
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key && \
    gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    rm /tmp/ros.key


RUN apt-get update && apt-get install -y \
  # opengl stuff
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  x11-apps \
  mesa-utils \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
  # other stuff
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  tig \
  # pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  gdbserver \
  htop \
  tmux \
  xterm \
  iputils-ping net-tools xvfb \
  python3-autopep8

  # Install ros distro testing packages
RUN apt install -y \
  ros-${ROS_DISTRO}-ament-lint \
  ros-${ROS_DISTRO}-launch-testing \
  ros-${ROS_DISTRO}-launch-testing-ament-cmake \
  ros-${ROS_DISTRO}-launch-testing-ros \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-cartographer-ros \
  ros-${ROS_DISTRO}-nav2-bringup  ros-${ROS_DISTRO}-navigation2

RUN apt install -y ros-${ROS_DISTRO}-gz-ros2-control ros-${ROS_DISTRO}-gazebo-ros2-control

# Install palanteer
RUN apt-get update && apt-get install -y \
    libdw-dev libunwind-dev     libgl1-mesa-glx libgl1-mesa-dri mesa-utils     && rm -rf /var/likb/apt/lists/*

RUN cd /opt && git clone https://github.com/dfeneyrou/palanteer && \
    cd palanteer && \
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Install PIP requirements
COPY requirements.txt /tmp/
RUN python3 -m pip install -r /tmp/requirements.txt

# Install supl dependencies
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    gcc cmake libssl-dev make ninja-build ranger \
    && rm -rf /var/lib/apt/lists/*

RUN rm -rf /var/lib/apt/lists/*
RUN rosdep init || echo "rosdep already initialized"

RUN mkdir -p /opt/dependencies_ws/src
# Build ROS 2 source dependencies

# cd /opt/dependencies_ws
WORKDIR /opt/dependencies_ws
COPY ros_dependencies.repos ros_dependencies.repos
RUN vcs import src < ros_dependencies.repos
RUN apt-get update && \
    rosdep update && \
    rosdep install -q -y --from-paths src --ignore-src && \
    rm -rf /var/likb/apt/lists/*

#  building dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /home/ros/src/

RUN echo "if [ -f /home/ros/install/setup.bash ]; then source /home/ros/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f /usr/share/gazebo/setup.sh ]; then . /usr/share/gazebo/setup.sh; fi" >> /home/ros/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MASTER_URI=http://0.0.0.0:12345" >> /home/ros/.bashrc
RUN echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/ros/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/usr/share/gazebo-11/models" >> /home/ros/.bashrc
RUN echo "source /home/ros/src/install/setup.bash" >> /home/ros/.bashrc
RUN echo "source /opt/dependencies_ws/install/setup.bash" >> /home/ros/.bashrc
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/ros/.bashrc
RUN echo "alias rosbuild='colcon build --merge-install --symlink-install --cmake-args -Wno-dev'" >> /home/ros/.bashrc

RUN sudo apt update && \
    sudo apt install ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-rqt-tf-tree silversearcher-ag -y && \
    sudo apt autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

COPY . /home/ros/src/
RUN chown -R ros:ros  /home/ros/src/
USER ros
RUN pre-commit install


