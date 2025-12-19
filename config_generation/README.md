# Config Generation

When building the workspaces, the generation scripts found in the scripts/ directory are run. These automatically generate the `bt.xml`, `control.yaml`, `gz_bridge.yaml`, `nav2_params.yaml`, as well as validate the setup of the `ROBOTS` list in the `config.sh` script. This is done to automatically set up the ROS2 controllers, navigation stack parameters, and other /topics depending on what robots and namespaces are spawned.

Every time the workspace is built, the `generate_XXX` files print that they were generated, and at the top of each generated file, a comment specifies from where the auto-generation occurred. As of now, the parameter files are saved in the package in which they're used instead of directly in the share/ directory to increase code readability for new users.

Explanation of the different generate_XXX files:

- [generate_bt_xml.py](scripts/generate_bt_xml.py): The bt_navigator needs the namespace of the robot it is checking conditions for.
- [generate_control_yaml.py](scripts/generate_control_yaml.py): The ros2_controller maps namespaced /cmd_vel topics to wheel_joint movements.
- [generate_gz_bridge_yaml.py](scripts/generate_gz_bridge_yaml.py): Some gz_topics need to be mapped to ROS2 topics.
- [generate_humanoid_control_yaml.py](scripts/generate_humanoid_control_yaml.py): The ros2_controller maps namespaced /cmd_vel topics to wheel_joint movements for humanoids.
- [generate_humanoid_nav2_params_yaml.py](scripts/generate_humanoid_nav2_params_yaml.py): The individual frame IDs and some ROS2 topics are needed to define humanoids and their corresponding maps.
- [generate_nav2_params_yaml.py](scripts/generate_nav2_params_yaml.py): The individual frame IDs and some ROS2 topics are needed to define robots and their corresponding maps.
- [validate_humanoids.py](scripts/validate_humanoids.py): A converter from string to list of dicts including a sanity check to throw errors if the HUMANOIDS variable is wrongly configured.
- [validate_robots.py](scripts/validate_robots.py): A converter from string to list of dicts including a sanity check to throw errors if the ROBOTS variable is wrongly configured.
