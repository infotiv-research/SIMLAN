## gazebo_ros2_control

```
[gzserver-1] [ERROR] [1727951819.671190014] [jackal.gazebo_ros2_control]: controller manager doesn't have an update_rate parameter
```

No solution

## OpenAL

```
[gzserver-1] [Err] [OpenAL.cc:84] Unable to open audio device[default]
```

Related to support for audio inside docker container. It will not be resolved

## Dark simulation

Make sure that nvidia driver is installed correctly

### Nvidia GPU support

To improve speed you can enable the support for nvidia GPU. To use nvidia GPU make sure [`nvidia-cuda-toolkit`](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) is installed. Here is a summary of how to install it on Ubuntu 24.04:

```

$ curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

$ sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list

$ sudo apt-get update

$ sudo apt-get install -y nvidia-container-toolkit

$ sudo nvidia-ctk runtime configure --runtime=docker
INFO[0000] Config file does not exist; using empty config
INFO[0000] Wrote updated config to /etc/docker/daemon.json
INFO[0000] It is recommended that docker daemon be restarted.

$ sudo systemctl restart docker
$ sudo nvidia-ctk runtime configure --runtime=containerd
INFO[0000] Using config version 1
INFO[0000] Using CRI runtime plugin name "cri"
WARN[0000] could not infer options from runtimes [runc crun]; using defaults
INFO[0000] Wrote updated config to /etc/containerd/config.toml
INFO[0000] It is recommended that containerd daemon be restarted.
```

To check for correct installation docker's nvidia runtime:

```
$ docker info|grep -i runtime
 Runtimes: nvidia runc
 Default Runtime: runc
```

Otherwise you get the following error message in vscode: `Error response from daemon: unknown or invalid runtime name: nvidia`

## Command failed: docker compose

If you have any issue with docker incompatibility (e.g. `Error: Command failed: docker compose ...`), make sure that `docker compose` or `docker-compose` is set correctly in the setting.
![dev container in vscode](resources/dev-container-config.png)

In `docker-compose.yaml`, uncomment the `factory_simulation_nvidia` section:

```
  factory_simulation_nvidia:
    <<: *research-base
    container_name: factory_simulation_nvidia
    runtime: nvidia
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: "all"
              capabilities: [compute,utility,graphics,display]
```

and update `.devcontainer/devcontainer.json`:

```
{
    "name": "ROS2 RESEARCH CONTAINER",
    "dockerComposeFile": "../docker-compose.yaml",
    "service": "factory_simulation_nvidia
...
```

## Missing nvidia docker runtime

Solution: https://stackoverflow.com/questions/59008295/add-nvidia-runtime-to-docker-runtimes

## docker issues

Make sure docker is installed correctly by following these two instructions:

- [Install docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
- [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/)

## Advanced options:

To record camera images for available cameras we use a simple python code [./camera_utility/camera_subscriber.py](./camera_utility/camera_subscriber.py) that continuously record camera images in `camera_utility/camera_data/` :

```bash
./control.sh cam_dump
```

## Advanced features

### Running ROS2 commands:

To avoid conflict between ros nodes in the same network, after each build a new random ROS2 domain is created. This means that you need to adjust this random domain before running any ros2 commands. For convenience you can use

```bash
./control.sh cmd YOUR_ROS2_COMMANDS
```

Here is an example of extracting tf2 hierarchy

```
./control.sh cmd ros2 run tf2_tools view_frames
```

These features and commands are under development and not fully supported yet and therefore are subject to change.

**Cartographer**: With both Gazebo and rviz running you can start creating a map of a robots surroundings

To start: `./control.sh cartographer`

The jackal can then be controlled with the computer keyboard by running:

```bash
./control.sh teleop_jackal
```

To record one screenshot after use:

```bash
./control.sh screenshot 164
```

The result will be stored in `./camera_utility/camera_data/`.

To record ros messages in ROS bag files to replay the scenario later:

```bash
./control.sh ros_record
```

To replay the last rosbag recording:

```bash
./control.sh ros_replay
```

To test the unit tests before pushing new codes:

```bash
./control.sh test
```

## Overview of Project Structure

- `README.md` : you are reading this file

- [`control.sh`](control.sh) : control script as a shortcut to run different scripts

- [`simulation/`](simulation/) : models, urdf and launch script for objects and agents in the gazebo simulator

  - [`README.md`](simulation/README.md) : simulation and warehouse specification
  - `infobot_agent/`
  - `object_mover/`
  - `raw_models/`
    - [`README.md`](simulation/raw_models/README.md) : building Gazebo models (Blender/Phobos)
    - [`objects/README.md`](simulation/raw_models/objects/README.md) : objects specifications
    - [`warehouse/README.md`](simulation/raw_models/warehouse/README.md) : warehouse specification
  - `simlan_gazebo_environment/`
  - `static_agent_launcher/`

- [`camera_utility/`](camera_utility/): to positions cameras in simulator and camera_utility images from simulation

  - [`aruco_localization/`](camera_utility/aruco_localization/) : ros2 package allowing camera to detect aruco code.

  - [`camera_data/`](camera_utility/camera_data/) : camera data are stored here

  - [`extrinsic/`](camera_utility/extrinsic/) : extrinsic camera calibration

  - [`intrinsic/`](camera_utility/intrinsic/) : intrinsic camera calibration

  - [`projection.ipynb`](camera_utility/projection.ipynb) : how to use camera calibration to project images to the ground plane

  - [`camconf2xacro.sh`](camera_utility/camconf2xacro.sh): to update camera calibration in simulator

  - [`calibration.py`](camera_utility/calibration.py) : to load and convert OpenCV calibration file

  - [`camera_subscriber.py`](camera_utility/camera_subscriber.py) : script to read images from ROS2 topics

  - [`README.md`](camera_utility/README.md) : to learn what camera_utility is done on images

  - `rviz_config.rviz`

- [`control/`](control/): to control the agents

  - `infobot_navigation2/`
  - `infobot_teleop/`
  - `maps/`
  - `nav2_commander/`

- [`resources/`](resources) : images , videos and documents are placed here (no code should be stored here)

- [`deployment/`](deployment) : CI/CD scripts for jenkins and k8s

- [`docker-compose.yaml`](docker-compose.yaml) : disable and enable nvidia gpu

- [`ISSUES.md`](ISSUES.md) : please check known issues before reporting any issue

- [`requirements.txt`](requirements.txt) : python (pip) dependencies

- [`CHANGELOG.md`](CHANGELOG.md)

- [`CREDITS.md`](CREDITS.md)

- `LICENSE`

- [`BUILD.md`](BUILD.md) : to build the project outside vscode
