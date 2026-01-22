###########################################
# Base image
###########################################
FROM althack/ros2:jazzy-cuda-gazebo


##########################################
# Invalid key signature fix
# https://askubuntu.com/questions/1341378/invalid-signature-error-for-ros-repository-while-trying-to-…
##########################################
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg



##########################################
#            Ubuntu pkgs
##########################################
RUN apt-get update && apt-get install -y \
    libgl1\
    libgl1-mesa-dri \
    x11-apps \
    mesa-utils \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    #pylint3 \
    python3.12-venv \
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
    python3-autopep8 \
    # other stuff \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    eog \
    tig


####################################################
#              Python environment                  #
####################################################
# Create a venv inside the container (optional)
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

RUN /opt/venv/bin/pip install catkin_pkg
RUN /opt/venv/bin/pip install "numpy<2.0"
RUN /opt/venv/bin/pip install empy
RUN /opt/venv/bin/pip install lark

RUN mkdir -p /opt/venv/lib/python3.12/site-packages/mediapipe/modules/pose_landmark \
 && chown -R ros:ros /opt/venv/lib/python3.12/site-packages/mediapipe/modules/pose_landmark \
 && chmod -R 775 /opt/venv/lib/python3.12/site-packages/mediapipe/modules/pose_landmark
########################################
#            Ros2 packages             #
########################################
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-launch-testing \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-launch-testing-ros \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-resources-panda-description \
    ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
    ros-${ROS_DISTRO}-moveit-py \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-twist-stamper \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-launch-pytest \
    && rm -rf /var/lib/apt/lists/*



############# HUMANOID

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-launch-testing \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-launch-testing-ros \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-resources-panda-description \
    ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
    ros-${ROS_DISTRO}-moveit-py \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*




#############################
# ROSDEP install and Update.
#############################
# Install supl dependencies
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    gcc cmake libssl-dev make ninja-build ranger \
    && rm -rf /var/lib/apt/lists/*

RUN rm -rf /var/lib/apt/lists/*
RUN rosdep init || echo "rosdep already initialized"
RUN mkdir -p /opt/dependencies_ws/src

# Build ROS 2 source dependencies
WORKDIR /opt/dependencies_ws
COPY ros_dependencies.repos ros_dependencies.repos
RUN vcs import src < ros_dependencies.repos
COPY ./scenario_utility/scenario_execution ./src/scenario_execution
RUN apt-get update && \
    rosdep update && \
    rosdep install -q -y --from-paths src --ignore-src && \
    rm -rf /var/likb/apt/lists/*

# building dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release




WORKDIR /home/ros/src/

RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MASTER_URI=http://0.0.0.0:12345" >> /home/ros/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> /home/ros/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/usr/share/gazebo-11/models" >> /home/ros/.bashrc

RUN echo "[[ -f /usr/share/gazebo/setup.sh ]]       && source /usr/share/gazebo/setup.sh" >> /home/ros/.bashrc
RUN echo "[[ -f /home/ros/src/install/setup.bash ]] && source /home/ros/src/install/setup.bash || echo 'Warning: the project is not built yet. run ./control.sh build'" >> /home/ros/.bashrc

RUN echo "source /opt/dependencies_ws/install/setup.bash" >> /home/ros/.bashrc
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/ros/.bashrc
RUN echo 'sudo chown -R ros:ros  /home/ros/src/' >> /home/ros/.bashrc
RUN echo 'if [[ "$OS" == *"win"* || "$OS" == *"Win"* ]]; then export DISPLAY=host.docker.internal:0.0; fi' >> /home/ros/.bashrc
RUN echo 'if [ -f /home/ros/src/config.sh ]; then trap '"'"'source /home/ros/src/config.sh; export ROS_DOMAIN_ID'"'"' DEBUG; fi' >> /home/ros/.bashrc
RUN echo 'pre-commit install' >> /home/ros/.bashrc
USER ros

# With the following:
# RUN echo 'if [ -f /home/ros/src/config.sh ]; then trap '"'"'source /home/ros/src/config.sh; export ROS_DOMAIN_ID'"'"' DEBUG; fi' >> /home/ros/.bashrc
# It is impossible to change ROS_DOMAIN_ID manually, if that is needed do something similar to the following instead:
#RUN echo "if [ -f /home/ros/src/ROS_DOMAIN_ID.txt ]; then prev_ros_domain_id=\"\"; trap 'new_ros_domain_id=\$(cat /home/ros/src/ROS_DOMAIN_ID.txt); if [ \"\$new_ros_domain_id\" != \"\$prev_ros_domain_id\" ]; then export ROS_DOMAIN_ID=\"\$new_ros_domain_id\"; echo ROS_DOMAIN_ID = \$new_ros_domain_id; prev_ros_domain_id=\"\$new_ros_domain_id\"; fi' DEBUG; fi" >> /home/ros/.bashrc
