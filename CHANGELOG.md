# Changelog

## v5.1.5: ur10, refactor: Team/Tech Lead: Hamid Ebadi

- add simulation real time factor to config : Hamid Ebadi
- introducing UR10 & Robotiq gripper: Markus and Elias (experimental)
- RITA models: Markus and Elias (experimental)
- refactor control.sh : Hamid Ebadi
- refactor build scripts : Hamid Ebadi
- add dynorobotics documentation : Hamid Ebadi

### 5.1.4: Code cleanup (Feb 2026) - Sebastian Olsson (Dyno Robotics)

- Remove SPAWN_JACKAL variable. Use `./control.sh jackal` to spawn the jackal instead.
- Move `replay_data` directory inside the `simulation/` folder.
- Example trajectory and images inside `replay_data` to test the visualize_real_data package.
- Add static transform for visualizing images and markers while running prepare.launch.py.
- Integration test for scenario_manager.
- Remove jackal dependency in pallet_truck.
- Remove emojis and weird symbols from scenario_manager README.

### 5.1.3: Time filtering and initial heading for replaying data (Jan 2026) - Sebastian Olsson (Dyno Robotics)

- Retrieve code that was pushed on feature/scenario-replay.
- Able to filter out timestamps to process in the .json files with start_time and end_time parameters.
- Set initial heading for the orientation_faker.py with initial_heading parameter.
- More clear listing of all .json files in replay_data/ as a table.

### 5.1.2: Port Dyno Robotics features to Jazzy (Jan 2026) - Sebastian Olsson (Dyno Robotics)

- Add SPAWN_JACKAL variable in config.sh to spawn jackal (can also be spawned with `./control.sh jackal`).
- Re-add jackal to ros_dependencies.repos so it does not require to clone the repo inside the simulation folder.
- Remove scenario_execution from ros_dependencies.repos, install with apt in Dockerfile instead.
- Re-add pytrees in ros_dependencies.repos. Update version to 2.3.x.
- Add command in control.sh to run scenario_manager and run a collision case.
- Add `drop_nuke.py` file which runs on `./control.sh kill` due to kill command previously not killing all ros-related ghost processes.
- Add ground_truth odom publisher to pallet_trucks (used by scenario_manager package to determine speed and position of pallet_trucks).
- Add .gitignored replay_data folder so images and trajectories can be used more easily instead of saving inside the install/share directory which deleted all data if a rebuild was performed.
- Add `ignore_orientation` parameter when replaying a scenario.
- Add `rviz_config` variable in config.sh to easily change which rviz file the sim is started with.
- Minor docs/README update to scenario_manager and visualize_real_data package.

### 5.1.1: Cleanup and Documentation (Jan 2026) - Team/Tech Lead: Hamid Ebadi

- Headless gazebo: Pär Aronsson
- Major cleanup, restructuring and documentation: Hamid Ebadi
- Improve ml prediction pipeline: Pär Aronsson

### 5.0.0: Refactor MoCap pipeline and sensors (Dec 2025) - Team/Tech Lead: Hamid Ebadi

- Working demo video scenario: Hamid Ebadi, Pär Aronsson
- Refactor and improve prediction pipeline: Pär Aronsson
- Review and update the mocap architecture: Pär Aronsson, Hamid Ebadi
- Pipeline for prediction on real data, images, and videos: Pär Aronsson
- Improve motion player: Pär Aronsson
- New demo scenario: Pär Aronsson
- Unify "single" and "multi" pipeline: Pär Aronsson, Hamid Ebadi
- Documentation and presentation (mkdocs, pandas): Hamid Ebadi

### 4.0.0: Improvements (Nov 2025) - Team/Tech Lead: Hamid Ebadi

- System integration tests: Pär Aronsson
- Overall system diagrams (launch files and bringups): Anton Stigemyr Hill
- PyTorch model for humanoid MoCap: Pär Aronsson
- Improved documentation and AutoGluon configuration: Marwa Naili
- Demo scenario: Anton Stigemyr Hill
- Semantic segmentation, depth, color camera sensors: Anton Stigemyr Hill, Pär Aronsson
- Humanoid navigation: Anton Stigemyr Hill
- Rework GPSS/ArUco navigation: Anton Stigemyr Hill

### 3.0.0: Jazzy, humanoid (Oct 2025) - Team/Tech Lead: Hamid Ebadi

- Collision sensor: Anton Stigemyr Hill
- Automatically generated parameter-files for robot_agents and world fidelity: Anton Stigemyr Hill
- Shared map and obstacle generation for robot_agents: Anton Stigemyr Hill
- Geofencing and safety stop: David Espedalen
- Humanoid motion capture: Casparsson and Siyu Yi, Hamid Ebadi
- Humanoid integration (pymoveit2 to moveit_py): Siyu Yi, Pär Aronsson
- Mocap (humanoid) with multi-camera training pipeline: Siyu Yi, Pär Aronsson
- Panda arm integration: Pär Aronsson
- Dyno-robotics trajectory replay integration, scenario: Sebastian Olsson

### 2.1.0: Namespace cleanup and multi-agent navigation (Aug 2025) - Team/Tech Lead: Hamid Ebadi

- Implemented generic robot-agent control supporting multiple meshes (e.g., pallet truck, forklift) and distinct ArUco IDs: Pär Aronsson
- Robot-agent and navigation2 (nav2) namespace: Pär Aronsson
- Multi-agent navigation capabilities using GPSS: Pär Aronsson

### 2.0.0: GPSS (May 2025) - Team/Tech Lead: Hamid Ebadi

- Upgrade Gazebo from classic to ignition, adapting new sensors: Nazeeh Alhosary
- ArUco_localization, added localization and nav2 to new robot: Pär Aronsson
- Navigation: Pär Aronsson
- Bird's Eye View ros package: Converting projection.ipynb to camera_bird_eye_view: Hamid Ebadi, Pär Aronsson
- Deprecated: InfoBot, replaced with Pallet_truck (based on Jackal by Clearpath Robotics)

### 1.0.6: Collision (Feb 2025)

- Add scenario execution library for ros2
- Add action servers for set_speed, teleport_robot and collision
- Add Node for TTC calculation
- Add package for custom messages

### 1.0.5: Delivery 4 (Dec 2024)

- D3.5 Disentanglement: One-parameter Object Movements.
- Trajectory visualization: Hamid Ebadi
- feat: added DynoWaitFor to make sure Jackal is launched last (synchronisation issue)
- simlan_bringup pkg created: Christoffer Johannesson

### 1.0.4: Delivery 3 (Oct 2024)

- SMILE-IV and ArtWork projects contributions
- Updated camera extrinsic, fixing OpenCV camera calibration to Gazebo simulator conversion: Erik Brorsson, Volvo
- D3.4 Out distribution data collection: Hamid Ebadi
- D3.3 In distribution data collection: Hamid Ebadi
- D3.2 Images for stitching: Hamid Ebadi
- D3.1 Dataset draft for HH: Hamid Ebadi
- CI/CD and Kubernetes integration by Filip Melberg, Vasiliki Kostara
- Jackal integration: Christoffer Johannesson, Hjalmar Ruscck
- Docker/vscode
- Disable GPU support by default
- POC for camera projection to pixel coordinates (Jupyter notebook): Hamid Ebadi
- Camera image dump and image stitching: Hamid Ebadi

### 1.0.3: Jackal Robot (Mar 2024) - Christoffer Johannesson

- Added dyno fork of jackal repo from Clearpath Robotics.
- Updated to Humble, added bringup and support for namespacing. Jackal can be spawned in Gazebo and controlled through the keyboard.
- Added .devcontainer folder with Dockerfile and devcontainer.json to set up project container in VS Code.
- Added docker-compose to link all needed files and set environment variables.
- Added .vscode folder with settings and tasks for easy building of the project.
- Updated README with info on how to use Docker setup in VS Code, and some features to make it easy to share the same setup with others.
- Features include: python3 dependency install with pip, cloning of other git repositories and how to make changes to those repositories.

### 1.0.2: Delivery 2 (Feb 2024)

- Volvo warehouse 0.0.1: Hamid Ebadi
- Volvo camera calibration in Gazebo 0.0.1: Hamid Ebadi
- Integrate Infobot_agent 0.0.2: InfoBot differential-drive AMR (Autonomous Mobile Robot) URDF and ROS launcher (GOPAL and forklift): Hamid Ebadi
- Integrate Infobot_cartographer 2.1.5: cartographer for creating PGM maps
- Integrate nav2_commander 0.0.2: ROS package to command Infobot where the destination is: Hamid Ebadi
- Integrate Infobot_navigation2 2.1.5: Standard Nav2 stack launcher: Hamid Ebadi
- Integrate Infobot_teleop 0.0.2: Teleoperation for InfoBot

### 1.0.1: Delivery 1 (Dec 2023) - Team/Tech Lead: Hamid Ebadi

- Basic warehouse model 1.0.0: Anders Bäckelie
- CAD modelling (eur-pallet, boxes, shelf, support_pole, traffic-cone, steel_drum) 1.0.0: Jacob Rohdin
- Physics (collision, inertia), visuals and Gazebo compatible mesh creation 1.0.0: Anders Bäckelie
- Walking actor using scripted trajectories 1.0.0: Anders Bäckelie
- Infobot_Gazebo_environment 1.0.0: ROS2 launcher to start Gazebo world: Hamid Ebadi
- static_agent_launcher 1.0.0: Camera and ArUco tags: Hamid Ebadi
- camera-viewer 1.0.0: Python code to get Gazebo camera feed: Hamid Ebadi
