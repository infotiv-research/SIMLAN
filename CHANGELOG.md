### 2.0.1: Namespace cleanup and multi-agent navigation (Aug 2025) - Supervisor: Hamid Ebadi

- Generic robot-agent with different mesh(pallet_truck/forklift) and aruco id
- Robot-agent and nav2 namespace
- Multi-agent navigation using GPSS

### 2.0.0: GPSS (May 2025) - Supervisor: Hamid Ebadi

- Upgrade Gazebo from classic to ignition, adapting new sensors: Nazeeh Alhosary
- Aruco_localization, added localization and nav2 to new robot: Pär Aronsson
- Navigation: Pär Aronsson
- Bird's Eye View ros package: Converting projection.ipynb to camera_bird_eye_view: Hamid Ebadi, Pär Aronsson
- Deprecated: Infobot, replaced with Pallet_truck (based on Jackal by Clearpath Robotics)

### 1.0.6: Collision (Feb 2025)

- Add scenario execution library for ros2
- Add action servers for set_speed, teleport_robot and collision
- Add Node for TTC calculation
- Add package for custom messages

### 1.0.5: Delivery 4 (Dec 2024)

- D3.5 Disentanglement: One-parameter Object Movements.
- Trajectory visualization : Hamid Ebadi
- feat: added DynoWaitFor to make sure Jackal is launched last (synchronisation issue)
- simlan_bringup pkg created : Christoffer Johannesson

### 1.0.4: Delivery 3 (Oct 2024)

- SMILE-IV and ArtWork projects contributions
- Updated camera extrinsic, fixing OpenCV camera calibration to Gazebo simulator conversion : Erik Brorsson, Volvo
- D3.4 Out distribution data collection : Hamid Ebadi
- D3.3 In distribution data collection : Hamid Ebadi
- D3.2 Images for stitching : Hamid Ebadi
- D3.1 Dataset draft for HH : Hamid Ebadi
- CI/CD and Kubernetes integration by Filip Melberg, Vasiliki Kostara
- Jackal integration : Christoffer Johannesson
- Docker/vscode
- Disable GPU support by default
- POC for camera projection to pixel coordinates  (jupyter notebook) : Hamid Ebadi
- Camera image dump and image stitching : Hamid Ebadi

### 1.0.3: Jackal Robot (Mar 2024) - Christoffer Johannesson

- Added dyno fork of jackal repo from Clearpath Robotics.
- Updated to Humble, added bringup and support for namespacing. Jackal can be spawned in Gazebo and controlled through the keyboard.
- Added .devcontainer folder with Dockerfile and devcontainer.json to set up project container in VS Code.
- Added docker-compose to link all needed files and set environment variables.
- Added .vscode folder with settings and tasks for easy building of the project.
- Updated README with info on how to use Docker setup in VS Code, and some features to make it easy to share the same setup with others.
- Features includes: python3 dependency install with pip, cloning of other git repositories and how to make changes to those repositories.

### 1.0.2: Delivery 2 (Feb 2024)

- Volvo warehouse 0.0.1 : Hamid Ebadi
- Volvo camera calibration in Gazebo 0.0.1 : Hamid Ebadi
- Integrate Infobot_agent 0.0.2: InfoBot differential-drive AMR (Autonomous Mobile Robot) URDF and ROS launcher (GOPAL and forklift): Hamid Ebadi
- Integrate Infobot_cartographer 2.1.5: cartographer for creating PGM maps
- Integrate nav2_commander 0.0.2: ROS package to command Infobot where the destination is : Hamid Ebadi
- Integrate Infobot_navigation2 2.1.5: Standard Nav2 stack launcher : Hamid Ebadi
- Integrate Infobot_teleop 0.0.2: Teleoperation for InfotBot

### 1.0.1: Delivery 1 (Dec 2023) - Supervisor: Hamid Ebadi

- Basic warehouse model 1.0.0: Anders Bäckelie
- CAD modelling (eur-pallet, boxes, shelf, support_pole, traffic-cone, steel_drum) 1.0.0 : Jacob Rohdin
- Physics (collision, inertia), visuals and Gazebo compatible mesh creation 1.0.0: Anders Bäckelie
- Walking actor using scripted trajectories 1.0.0 : Anders Bäckelie
- Infobot_Gazebo_environment 1.0.0: ROS2 launcher to start Gazebo world : Hamid Ebadi
- static_agent_launcher 1.0.0: Camera and Aruco tags : Hamid Ebadi
- camera-viewer 1.0.0: Python code to get Gazebo camera feed : Hamid Ebadi
