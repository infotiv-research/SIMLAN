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
