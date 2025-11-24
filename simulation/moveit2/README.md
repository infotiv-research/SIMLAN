# Moveit Panda robot

This package contains code for a robot called ["Panda"](https://github.com/moveit/moveit_resources) which includes a description/urdf's  (see `panda_description/`).
Inside the `panda_moveit_config/` exists all code and config files that allow you to spawn a panda robot in gazebo and be able to plan and execute motions.
Inside `config/` the are several `*.yaml` files which acts as configurations, and many `*.srdf, *.urdfs *.xacro` files. These are all related to the same panda robot. The main description file is `panda.urdf.xacro`.

To run our package make sure that gazebo simulator is running. Then we can run the `panda_moveit_config/launch/demo.launch.py` which will do the following:

- Spawn a panda robot in gz and publish its `robot_description`.
- Start rviz with a preset moveit config
- Start the `move_group_node` which handles all planning and executing of motions.
- Load all controllers for panda so that it can be controlled in Gazebo

## Moveit motion planner using Python API

The folder[`custom_motion_planning_python_api`](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html) has scripting files that plans and executes motions for a panda robot. It uses the official moveit2 python API to achieve this.
In this package's `scripts/` folder exists these python files. Look at the demo scripts for inspiration. To run any of the scripts you run the launch file:

- `motion_planning_python_api_planning_scene.py` is an original demo from moveit2
- `motion_planning_python_api_tutorial.py` is an original demo from moveit2
- `demo_pick_and_place.py` is custom made.

`moveit2_py` package is not currently available for Humble but Jazzy supports it. The pre-requisites to run a motion planner script is to run these in parallel terminals: `gazebo sim & panda_moveit_config/launch/demo.launch.py`. When these to are finished initializing, run the motion script with: `motion_planning_python_api_tutorial.launch.py`
