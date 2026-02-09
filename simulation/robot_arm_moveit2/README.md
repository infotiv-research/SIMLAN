# Moveit Panda robot

This package contains code for a robot called ["Panda"](https://github.com/moveit/moveit_resources) which includes a description/URDFs (see `panda_description/`).
Inside the `panda_moveit_config/` exist all code and config files that allow you to spawn a panda robot in Gazebo and be able to plan and execute motions.
Inside `config/` there are several `*.yaml` files which act as configurations, and many `*.srdf, *.urdf, *.xacro` files. These are all related to the same panda robot. The main description file is `panda.urdf.xacro`.

To run our package, make sure that the Gazebo simulator is running. Then we can run the `panda_moveit_config/launch/demo.launch.py`, which will do the following:

- Spawn a panda robot in Gazebo and publish its `robot_description`.
- Start RViz with a preset MoveIt config.
- Start the `move_group_node`, which handles all planning and executing of motions.
- Load all controllers for panda so that it can be controlled in Gazebo.

## UR10 resources

The UR10 setup lives under `moveit_resources/`

- `ur_description/`: UR family URDFs, meshes, and parameter YAMLs.
  - `ur_description/urdf/ur.urdf.xacro` and `ur_description/urdf/ur_macro.xacro` are the main description entry points.
  - `ur_description/config/ur10/*.yaml` holds UR10-specific kinematics, limits, and visuals.
- `ur_moveit_config/`: MoveIt2 configuration, controllers, and launch files for UR10.
  - `ur_moveit_config/config/ur10.urdf.xacro` is the MoveIt/Gazebo description (includes Robotiq).
  - `ur_moveit_config/config/ros2_controllers.yaml` and `ur_moveit_config/config/gazebo_moveit_controllers.yaml` define controllers.
  - `ur_moveit_config/srdf/ur.srdf.xacro` contains the SRDF groups and gripper setup.
  - `ur_moveit_config/launch/demo.launch.py` is the main entry point for spawning + MoveIt.
- `robotiq_description/`: Robotiq 2F-140 gripper description and meshes.
  - `robotiq_description/urdf/robotiq_2f_140_macro.urdf.xacro` is the main gripper macro used by UR10.

## UR10 quickstart

1. Start Gazebo in a separate terminal:

- `./control.sh sim`

1. Spawn the UR10 + MoveIt stack (after Gazebo is already running):

- `./control.sh ur10`

3. Launch the UR10 pick-and-place script (after the UR10 stack is up):

- `ros2 launch motion_planning_python_api ur10_pick_and_place.launch.py`

## Moveit motion planner using Python API

The folder [`custom_motion_planning_python_api`](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html) has scripting files that plan and execute motions for a panda robot. It uses the official MoveIt2 Python API to achieve this.
In this package's `scripts/` folder exist these Python files. Look at the demo scripts for inspiration. To run any of the scripts, you run the launch file:

- `motion_planning_python_api_planning_scene.py` is an original demo from MoveIt2.
- `motion_planning_python_api_tutorial.py` is an original demo from MoveIt2.
- `demo_pick_and_place.py` is custom made.

The `moveit2_py` package is not currently available for Humble, but Jazzy supports it. The prerequisites to run a motion planner script are to run these in parallel terminals: `gazebo sim & panda_moveit_config/launch/demo.launch.py`. When these two are finished initializing, run the motion script with: `motion_planning_python_api_tutorial.launch.py`
