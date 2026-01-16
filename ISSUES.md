## Advanced Features

### Filtering log output

In `config.sh` you can set the level of logs you want outputted into the terminal. By default it is set to "info" to allow all logs. Possible values are: "debug", "info", "warn", and "error". Setting it to "warn" filters out all debug and info messages. Additionally, to filter out specific lines you can add the phrase you want filtered inside `log_blacklist.txt` and setting the `log_level` flag to "warn" or "error" will start filtering out all phrases found in the blacklist.

### Aruco detection

If you want to add the TF links between the cameras and the ArUco markers without running the `gpss` command, you can run the following command. This is primarily useful for debugging, as `gpss` runs this as well.

```bash
./control.sh aruco_detection
```

Finally, to view the bird's-eye perspective from each camera, run the following command and open `rviz`. Then, navigate to the left panel and under "Camera" change the Topic `/static_agents/camera_XXX/image_projected` to visualize the corresponding camera feed:

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

### Teleop

If you want to control any robot (pallet truck, humanoid, etc.) manually you can run the following command. Remember to specify what robot you want to control by adding its namespace as argument, i.e. `./control.sh teleop pallet_truck_1`

```bash
./control.sh teleop ${YOUR_ROBOT_NAMESPACE}
```

To move the humanoid around in the simulator

```bash
./control.sh teleop ${YOUR_HUMANOID_NAMESPACE}
```

### Scenarios

In [scenarios.sh](scenarios.sh) you can run predefined scenarios of the project. At the bottom of the file, commands are shown how to run it; otherwise each scenario is referenced by a number.
Currently there are 3 scenarios and to run them, run this command in the shell

Before running scenario 1 that uses GPSS cameras, make sure that `CAMERA_ENABLED_IDS` in `config.sh` has the list of GPSS cameras.

> Note: A previous successful setup where all scenarios worked used these cameras: '160 161 162 163 164 165 166 167 168 169 170'. If robots are not moving in your simulation, it might be that there is no camera looking at the robot.

```
./control.sh build
./scenarios.sh 1 # navigation using GPSS cameras (real time factor need to be updated)
./scenarios.sh 2 # Humanoid and robotic arm
./scenarios.sh 3 # Humanoid navigation without GPSS cameras (navigate_w_replanning_and_recovery_robot_agent_X.xml need to be updated )
./scenarios.sh 4 # Record the video demo
```

If you want to see the planning route for all agents, load `scenario_3_planning.rviz` in Rviz.

Keep in mind to change `real_time_factor` in [`simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro`](/simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro) to small values to slow down the simulator (e.g. 0.05-0.1) before building the project. `./control.sh build`

### Jackal

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

## ISSUES

### gazebo_ros2_control

```
[gzserver-1] [ERROR] [1727951819.671190014] [jackal.gazebo_ros2_control]: controller manager doesn't have an update_rate parameter
```

No solution.

### OpenAL

```
[gzserver-1] [Err] [OpenAL.cc:84] Unable to open audio device[default]
```

Related to support for audio inside a Docker container. It will not be resolved.

### Command failed: docker compose

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

### Missing nvidia docker runtime

Solution: https://stackoverflow.com/questions/59008295/add-nvidia-runtime-to-docker-runtimes

### Docker nvidia-container-cli: requirement error

If you get this error when building the docker container:

```
nvidia-container-cli: requirement error: unsatisfied condition: cuda>=12.6, please update your driver to a newer version, or use an earlier cuda container: unknown
```

A solution to try is first: re-install nvidia-container-toolkit. Or, if that does not work, update your nvidia-drivers.

### GLIBC_2 issue

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

### docker issues

Make sure docker is installed correctly by following these two instructions:

- [Install docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
- [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/)
