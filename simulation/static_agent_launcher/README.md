# Launching GPSS cameras

This package launches all the static agents, i.e., the cameras in the simulation.

When the workspace is built, the `camera_config.xacro` file is updated with camera_ids and their corresponding intrinsic and extrinsic values. This can be seen in the `build` function in `control.sh`.

The `camera_config.xacro` is then sent to the robot_state_publisher node and published to the /static_agent/robot_description topic. The ros_gz_sim package with the "create" executable then spawns the static_agents by subscribing to the /static_agent/robot_description topic and spawning each agent.

______________________________________________________________________

### gz_bridge for cameras

Different configurations of the gz_bridge node were tested and evaluated against the simulated RTF (real time factor). In the earlier setups, a gz_bridge node was set up for each camera_stream (image, depth, semantic) for each camera_id. Three configurations were tested where different numbers of nodes were set up and their corresponding RTFs were captured and averaged over five tries.

SN=single node for all cameras, 1NPC=1 node per camera_id, 1NPSPC = 1 node per stream per camera_id

| node setup | number of cameras | number of nodes | camera streams       | avg RTF |
| ---------- | ----------------- | --------------- | -------------------- | ------- |
| SN         | 9                 | 1               | image                | 17,9%   |
| 1NPC       | 9                 | 9               | image                | 16,7%   |
| SN         | 9                 | 1               | image depth semantic | 5,14%   |
| 1NPC       | 9                 | 9               | image depth semantic | 5,2%    |
| 1NPS       | 9                 | 81              | image depth semantic | 4,7%    |

It doesn't seem like a big difference, so we will stick with one node, i.e., SN.

In the future it can be decided whether to keep the gz_bridge node as is or to generate a gz_bridge_camera.yaml file like the other generation scripts (`config_generation`).
