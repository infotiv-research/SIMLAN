## gazebo_ros2_control

```
[gzserver-1] [ERROR] [1727951819.671190014] [jackal.gazebo_ros2_control]: controller manager doesn't have an update_rate parameter
```

No solution.

## OpenAL

```
[gzserver-1] [Err] [OpenAL.cc:84] Unable to open audio device[default]
```

Related to support for audio inside a Docker container. It will not be resolved.

## Command failed: docker compose

If you have any issue with docker incompatibility (e.g. `Error: Command failed: docker compose ...`), make sure that `docker compose` or `docker-compose` is set correctly in the settings.

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

## Docker nvidia-container-cli: requirement error

If you get this error when building the docker container:

```
nvidia-container-cli: requirement error: unsatisfied condition: cuda>=12.6, please update your driver to a newer version, or use an earlier cuda container: unknown
```

A solution to try is first: re-install nvidia-container-toolkit. Or, if that does not work, update your nvidia-drivers.

## GLIBC_2 issue

Sometimes the wrong `nvidia-container-toolkit` results in this issue below:

```
$/usr/lib/x86_64-linux-gnu/libc.so.6: version `GLIBC_2.38' not found (required by /usr/lib/x86_64-linux-gnu/libGLdispatch.so.0)
$/usr/lib/x86_64-linux-gnu/libc.so.6: version `GLIBC_2.38' not found (required by /usr/lib/x86_64-linux-gnu/libGLX.so.0)
```

If so, try downgrading `nvidia-container- toolkit`. You can use these commands:

```
$sudo apt-get remove --purge nvidia-container-toolkit nvidia-container-toolkit-base libnvidia-container*
$sudo apt-get update
$sudo apt-get install nvidia-container-toolkit=1.17.4-1 nvidia-container-toolkit-base=1.17.4-1 libnvidia-container1=1.17.4-1 libnvidia-container-tools=1.17.4-1
```

## docker issues

Make sure docker is installed correctly by following these two instructions:

- [Install docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
- [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/)

## Advanced options:

To record camera images for available cameras we use a simple Python code [./camera_utility/camera_subscriber.py](./camera_utility/camera_subscriber.py) that continuously records camera images in `camera_utility/camera_data/`:

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

**Cartographer**: With both Gazebo and rviz running you can start creating a map of a robot's surroundings.

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

To record ROS messages in ROS bag files to replay the scenario later:

```bash
./control.sh ros_record
```

To replay the last rosbag recording:

```bash
./control.sh ros_replay
```

To test the unit tests before pushing new code:

```bash
./control.sh test
```

## Jackal

If you want to control the Jackal, you add the following lines into the control.sh:

```bash
elif [[ "$*" == *"jackal_teleop"* ]]
then
    ros2 launch dyno_jackal_bringup keyboard_steering.launch.py
```

And then run:

```bash
./control.sh jackal_teleop
```

These ones did not work, so we put them here in issues

```bash
elif [[ "$*" == *"move_object"* ]]
then
    ros2 run object_mover move_object
```

```bash
elif [[ "$*" == *"scenario"* ]]
then
    ros2 launch scenario_execution_ros scenario_launch.py scenario:=simulation/scenario_manager/scenarios/test.osc
```
