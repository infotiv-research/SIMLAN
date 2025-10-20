# SIMLAN, Simulation for Indoor Multi-Camera Localization and Navigation (3.0.0)

This simulation environment, based on the Gazebo Ignition simulator and ROS2, resembles a Volvo trucks warehouse and serves as a playground for rapid prototyping and testing of systems that rely on multi-camera setup for perception, monitoring, localization or even navigation. This project is inspired by [GPSS (Generic photo-based sensor system)](https://www.volvogroup.com/en/news-and-media/news/2024/nov/ai-modern-manufacturing.html) that utilizes ceiling mounted cameras, deep learning and computer vision algorithms, and very simple transport robots.

![](/resources/logos/youtube.png)[GPSS demo](https://www.youtube.com/watch?v=DA7lKiCdkCc)



## SIMLAN Features

- Ignition Gazebo
- Library of assets
- Real-World Environment Inspired Design (camera position and warehouse layout)
- ROS 2 Interfaces (Humble and Jazzy)
- ArUco Marker Localization
- Simple GPSS (Generic photo-based sensor system) navigation
- Multi-Robot Localization and Navigation using Nav2
- Bird's-Eye View Projection
- Multi-Sensor Support (LiDAR, Camera, Semantic segmentation, Depth etc.)
- Geofencing for safe zones and safestop on collision
- Motion capture for Human-Robot Collaboration/Interaction (HRC/HRI)

Please click the youTube link below to view the SIMLAN demo video:

![](/resources/logos/youtube.png) [SIMLAN demo](ttps://www.youtube.com/watch?v=mhA51PPdABc)

![SIMLAN, Simulation for Indoor Multi-Camera Localization and Navigation](resources/demo.png)

Here are list of advantages of using SIMLAN Multi-Camera system

- Rapid prototyping and iteration of ML based algorithm. (e.g. Reinforcement Learning)
- Enhanced monitoring and coordination using bird eye view
- Simplified robot design and maintenance.
- Extendible with additional ML based vision systems
- Safety testing without physical Risk or Privacy concerns
- Scalable and reproducible testing (CI/CD)
- Cost-effective development


## Installation

### Dependencies

**Ubuntu 24.04:** use [DEPENDENCIES_Linux.md](DEPENDENCIES_Linux.md) to install docker and ensure that your linux user account has `docker` access.
*Attention*: Make sure to restart the computer (for the changes in group membership to take effect.) before proceeding to the next step.

**Windows 11:** use [DEPENDENCIES_Windows.md](DEPENDENCIES_Windows.md) to install dependencies.


**Production environment**: follow installation procedure used in [.devcontainer/Dockerfile](.devcontainer/Dockerfile) to install dependencies.

**Development environment**: to improve collaboration we use vscode and docker as explained in [this instruction](https://www.allisonthackston.com/articles/docker-development.html) and [docker files](https://github.com/athackst/dockerfiles).
Install Visual Studio Code (VS Code) and open the project folder. VS Code will prompt you to install the required extension dependencies.
Make sure the `Dev containers` extension is installed. Reopen the project in VS Code, and you will be prompted to rebuild the container. Accept the prompt, this process may take a few minutes.
Once VS Code is connected to Docker (as shown in the image below), open the terminal and run the following commands:

![](/resources/logos/youtube.png)  [Installation and how to start](https://www.youtube.com/watch?v=DgJXlsXUa-w)

![dev container in vscode](resources/vscode.png)

(if you don't see this try to build manually in vscode by pressing `Ctrl + Shift + P` and select `Dev containers: Rebuild and Reopen in container`.
)

#### Quick Start

The best place to learn about the various features, start different components, and understand the project structure is [`./control.sh`](./control.sh).

*Attention*: The following commands (using `./control.sh`) are executed in a separate terminal tab inside _vscode_.

To kill all the relevant process (related to gazebo, ros2), delete build files, delete recorded images and `rosbag` files using the following command:

```bash
./control.sh clean
```

To clean up and build the ros2 simulation

```bash
./control.sh build
```

## GPSS controls

It is possible for the cameras to detect ArUco markers on the floor and publish their location to TF, both relative to the camera, and the ArUcos transform from origin. The package [./camera_utility/aruco_localization](./camera_utility/aruco_localization) contain the code for handling ArUco detection.

You can also use nav2 to make a robot_agent (that can be either robot/pallet_truck) navigate by itself to a goal position. You can find the code in [simulation/pallet_truck/pallet_truck_navigation](simulation/pallet_truck/pallet_truck_navigation)


![](/resources/logos/youtube.png) [GPSS inside SIMLAN demo](https://www.youtube.com/watch?v=_UhRFR-L9iQ)


**Run these three in separate terminals**

```bash
./control.sh gpss # spawn the simulation, robot_agents and GPSS ArUco detection
./control.sh nav  # spawn map server, and separate nav2 stack in a separate namespace for each robot_agent
./control.sh send_goals # send navigation goals to nav2 stack for each robot_agent
```


If you want to control any robot manually you can run the following command. Remember to specify what robot you want to control by adding its namespace as argument, i.e. `./control.sh teleop pallet_truck_1`

```bash
./control.sh teleop ${YOUR_ROBOT_NAMESPACE}
```

If you want to record any of your topics during the tests you can run the following command. Change the topic in the control.sh script: `ros2 bag record /topic` to whatever topic you want to record.

```bash
./control.sh ros_record
```

To replay your latest recorded rosbag run the following command:

```bash
./control.sh ros_replay
```

If you want to do a camera dump and save the image from each camera as a .png run the following command. The images will appear at `/src/camera_utility/camera_number`.

```bash
./control.sh camera_dump
```

If you want to take a screenshot of one of the cameras view, run the following command. Replace `###` with the camera you want to take a screenshot of. (163, 164, 165 or 166)

```bash
./control.sh screenshot ###
```

If you want to add the tf links between the cameras and the ArUco markers without running the `gpss` command you can run the following command. This is not that usable as the `gpss` run this as well, but it can be good for debugging.

```bash
./control.sh aruco_detection
```

Finally, to view the bird's-eye perspective from each camera, run the following command and open `rviz` Then, navigate to the scroll menu to the left, and under "Camera" change the Topic `/static_agents/camera_XXX/image_projected` topic to visualize the corresponding camera feed:

```bash
./control.sh birdeye
```

## RITA controls (humanoid, robotic arm)

![](/resources/logos/youtube.png) [Humanoid/Arm demo](https://www.youtube.com/watch?v=EiCNiPeifPk)


> The dataset, input and outputs are in `` directory
>
To select an empty world run:

```bash
./control.sh world empty
```

To create a humanoid dataset ( paired pose data, motion data and reference images) in the `DATASET/TRAIN` directory:

```bash
./control.sh dataset TRAIN
```

The resulting files are stored as parallel `.json` files in multi-camera folders `camera_*/pose_data`, `camera_*/motion_data`, `camera_*/pose_images`.
To avoid creation of `pose_images` that are only used as the ground truth and debugging (specially if you are building your training data), comment out then call to `self.save_pose_image()` in `camera_viewer.py`.

Finally to merge them all in tabular `.csv` file run the following command.
```
./control.sh convert2csv
```
Results in merging:
- `DATASET/TRAIN` to `DATASET/train.csv`
- `DATASET/EVAL` to `DATASET/eval.csv`

You can replay_motion each motion data separately: `./control.sh replay_motion DATASET/EVAL/motion_data/AAAAAAA_motion.json`


> Keep in mind that we use a separate virtual environment to install machine learning related pip packages called `mlenv`  with a separate [`requirements.txt`](pose_to_motion/MLrequirements.txt). Build the environment first using `./control.sh mlenv`


This step reads data from `DATASET/train.csv` and trains our model:

```
./control.sh train
```
The resulting model is saved in `models/`.

The following command uses MediaPipe to create pose data from an image or a video in `input/` to populate `output/` with mediapipe lankmark pose and images.

```
./control.sh image_pipeline
./control.sh video_pipeline
```
To use the model on each generated pose run the following command

```
./control.sh predict output/
```

Alternatively, you can also run similar command to do prediction on the validation data. Using evaluation dataset opens up a image viewer with the ground truth mediapipe detection as well to quickly check the performance of our ml model:
```
./control.sh predict DATASET/EVAL
```
#### Arm controls

Spawn the Panda arm inside SIMLAN and instruct it to pick and place a box around with the following commands:

```
./control panda
./control plan_motion
./control pick
```

## Advanced options

See [ISSUES.md](ISSUES.md) to learn about additional advanced options and to check known issues before reporting any issue or requesting new features. To start the project **without NVIDIA GPU** please comment out these lines in `docker-compose.yaml` as shown below:

```bash
  #   runtime: nvidia
  #
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

`camera_enabled_ids` specifies which cameras are enabled in the scene for ArUco code detection and birdeye view.


### Customized startup

In `config.sh` it is possible to customize your scenarios. From there you can edit what world you want to run, how many cameras you want enabled, and also edit Humanoid related properties. Modifying these variables are preferred, rather than modifying the `control.sh` file.

### World fidelity

in the `config.sh` script, you can adjust the world fidelity

The active worlds are:

|arguments | configuration |
|----------|---------------|
| `default`|Contains the default world with maximum objects |
| `medium` |Based on default but boxes are removed          |
| `light`  |Based on medium but shelves are removed         |
| `empty`  |Everything except the ground is removed         |

### Filtering log output

In `config.sh` you can set the level of logs you want outputed into the terminal. Per default it is set to "info" to allow all logs. Possible values are: "debug", "info", "warn", and "error". Setting it to "warn" filters out all debug and info messages. Additionally, to filter out specific lines you can add the phrase you want filtered, inside of `log_blacklist.txt` and setting the `log_level` flag to "warn" or "error" will start filtering out all phrases found in the blacklist.



### Older versions

- **Gazebo Classic (Gazebo11)** has reached end-of-life (EOL). An earlier version of this repository, which uses Gazebo Classic, can be found in the [gz_classic_humble branch](https://github.com/infotiv-research/SIMLAN/tree/gz_classic_humble).
- **ROS2 humble & Gazebo ignition**, an earlier version of this repository, which uses ROS2 humble and Gazebo ignition can be found in https://github.com/infotiv-research/SIMLAN/tree/ign_humble.


## Documentation

Learn more about the project by reading these documents:

- [Control script as a shortcut to run different scripts](control.sh)
- [Marp Markdown Presentation](PRESENTATION.md)
- [Pallet Truck Navigation Documentation](simulation/pallet_truck/pallet_truck_navigation/README.md)
- [Camera positioning (Extrinsic/Intrinsic calibrations) and utilities](camera_utility/)
- [Humanoid pose2motion Utilities](humanoid_utility/README.md)
- [`simulation/`](simulation/): ROS2 packages
  - [Simulation and Warehouse Specification (fidelity)](simulation/README.md)
  - [Building Gazebo models (Blender/Phobos)](simulation/raw_models/README.md)
  - [Objects Specifications](simulation/raw_models/objects/README.md)
  - [Warehouse Specification](simulation/raw_models/warehouse/README.md)
  - [Aruco Localization Documentation](simulation/aruco_localization/README.md)
  - [humanoid_robot Simulation](simulation/humanoid_robot/)
  - [Geofencing and Collision safe stop](simulation/bt_failsafe/README.md)
  - [Visualize Real Data](simulation/visualize_real_data/README.md) **requires data from Volvo**
- [`CHANGELOG.md`](CHANGELOG.md)
- [`CREDITS.md`](CREDITS.md)
- [`LICENSE` (apache 2)](LICENSE)

## Research Funding

This work was carried out within these research projects:

- The [SMILE IV](https://www.vinnova.se/p/smile-iv/) project financed by Vinnova, FFI, Fordonsstrategisk forskning och innovation under the grant number 2023-00789.
- The EUREKA [ITEA4](https://www.vinnova.se/p/artwork---the-smart-and-connected-worker/) ArtWork - The smart and connected worker financed by Vinnova under the grant number 2023-00970.


INFOTIV AB | Dyno-robotics | RISE Research Institutes of Sweden | CHALMERS | Volvo Group
------------ |  ------------  | ------------ | ------------ | ------------
![](resources/logos/INFOTIV-logo.png)  |  ![](resources/logos/dyno-robotics.png) | ![](resources/logos/RISE-logo.png)  | ![](resources/logos/CHALMERS-logo.png) | ![](resources/logos/volvo.jpg)

[SIMLAN](https://github.com/infotiv-research/SIMLAN) project is started and is currently maintained by [Hamid Ebadi](https://github.com/ebadi).
