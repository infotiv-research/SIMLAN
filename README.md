# SIMLAN, Simulation for Indoor Multi-Camera Localization and Navigation (2.0.0)

This simulation environment, based on the Gazebo Ignition simulator and ROS2, resembles a Volvo trucks warehouse and serves as a playground for rapid prototyping and testing indoor multi-camera localization, positioning, and navigation algorithms. While this environment can be utilized for Multi-Sensor SLAM (Simultaneous Localization and Mapping) using cameras, IMUs, GPS, LiDAR, and radar *mounted on the robot*, the focus of this project is *not* on mapping but operating within a *fixed building layout* and using fixed cameras *mounted on the ceiling*. This project is inspired by [GPSS (Generic photo-based sensor system)](https://www.youtube.com/watch?v=DA7lKiCdkCc) that utilizes ceiling mounted cameras, deep learning and computer vision algorithms, and very simple transport robots.

Please click the YouTube link below to view the demo video:

[![SIMLAN, Simulation for Indoor Multi-Camera Localization and Navigation](https://img.youtube.com/vi/mhA51PPdABc/0.jpg)](https://www.youtube.com/watch?v=mhA51PPdABc)

## Installation

To improve collaboration in development environment we use vscode and docker as explained in [this instruction](https://www.allisonthackston.com/articles/docker-development.html) using these [docker files](https://github.com/athackst/dockerfiles). For production environment follow installation procedure used in [.devcontainer/Dockerfile](.devcontainer/Dockerfile) to install dependencies.

To summarize, you can use these commands (tested on Ubuntu 24.04) to install docker and ensure that your linux user account has `docker` access:

```
sudo apt install curl git
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

ATTENTION: You need to **logout** and then log back in for the changes in group membership to take effect.

Install Visual Studio Code (VS Code) and open the project folder. VS Code will prompt you to install the required extension dependencies.
Make sure the `Dev containers` extension is installed. Reopen the project in VS Code, and you will be prompted to rebuild the container. Accept the prompt, this process may take a few minutes.
Once VS Code is connected to Docker (as shown in the image below), open the terminal and run the following commands:

![dev container in vscode](resources/vscode.png)
(if you don't see this try to build manually in vscode by pressing `Ctrl + Shift + P` and select `Dev containers: Rebuild and Reopen in container`.
)

## Quick start

Run the following commands on a separate terminal in the **host** machine (**not** inside Visual Studio Code terminal):

```
xhost +local:docker
```

To kill all the relevant process (related to gazebo, ros2), delete build files, delete recorded images and rosbag files using the following command:

*Attention*: in a separate terminal tab inside _vscode_.

```bash
./control.sh clean
```

To clean up and build the ros2 simulation

```bash
./control.sh build
```

It is possible for the cameras to detect aruco markers on the floor and publish their location to TF, both relative to the camera, and the arucos transform from origin. The package [./camera_utility/aruco_localization](./camera_utility/aruco_localization) contain the code for handling aruco detection.

You can also use nav2 to make a robot/pallet_truck navigate by itself to a goal position. You can find the code in [simulation/pallet_truck/pallet_truck_navigation](simulation/pallet_truck/pallet_truck_navigation)

```bash
./control.sh gpss
```

Finally, to view the bird's-eye perspective from each camera, run the following command and open `rviz` Then, navigate to the `/static_agents/camera_XXX/image_projected` topic to visualize the corresponding camera feed:

```bash
./control.sh birdeye
```

## Advanced options

See [ISSUES.md](ISSUES.md) to learn about additional advanced options. To start the project **without NVIDIA GPU** please comment out these lines in `docker-compose.yaml` as shown below:

```bash
  #   runtime: nvidia

  # factory_simulation_nvidia:
  #  <<: *research-base
  #  container_name: factory_simulation_nvidia
  #  runtime: nvidia
  #  deploy:
  #    resources:
  #      reservations:
  #        devices:
  #          - driver: nvidia
  #            count: "all"
  #            capabilities: [compute,utility,graphics,display]
```

### Gazebo Classic

Gazebo Classic (Gazebo11) has reached end-of-life (EOL). An earlier version of this repository, which uses Gazebo Classic, can be found in the [gz_classic_humble branch](https://github.com/infotiv-research/SIMLAN/tree/gz_classic_humble).

## Research Funding

This work was carried out within these research projects:

- The [SMILE IV](https://www.vinnova.se/p/smile-iv/) project financed by Vinnova, FFI, Fordonsstrategisk forskning och innovation under the grant number 2023-00789.
- The EUREKA [ITEA4](https://www.vinnova.se/p/artwork---the-smart-and-connected-worker/) ArtWork - The smart and connected worker financed by Vinnova under the grant number 2023-00970.
