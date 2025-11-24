# Aruco Localization Package

This is the `aruco_localization` package, which runs a ros2 node that takes images from camera topics `CAMERA_X/camera_info` and `CAMERA_X/image_raw`, process the aruco code, and publishes the result in the `/TF` of the detected ArUco marker, mimicking Volvo's logic for tracking robot positions and generating navigation routes. A launch file is provided to run multiple nodes, with each node dedicated and assigned to a single camera.
`camera_enabled_ids` variable found in `config.sh` is used to control which cameras are enabled.

**Note:** The marker link is named based on the camera. For example, if camera `164` detects an ArUco marker with ID of `12`, the marker link in `/tf` is named `camera_164_marker_12`. This is not the final link that is used. The result from different cameras are accumulated and merged as a single frame with parent as `robot_agent_12/odom` and child `robot_agent_12/baselink`.

**Note:**  The marker ID is tightly linked with the name for the robots, meaning an aruco with marker_ID=1 will detect the robot with name: {namespace}/{marker_ID}. For example 'robot_agent_1'.

## Important  points to pay attention to:

- If two cameras are pointed at the same ArUco code, the system does not alternate between them, but instead shows the midpoint between the two detections.
- Inside the aruco_node, a callback logs the relative position of the newly created `aruco_link` from the `base_link`.
- Multiple cameras can be linked to the same ArUco code simultaneously.

## Important launch files

- `aruco_detection_node.py`: Handles ArUco detection and publishes the TF between the camera and marker. (e.g. `camera_164_marker_12`)

- `aruco_pose_pub.py`: A new node that listens to all transforms of `camera_164_marker_12` as input, and outputs the transform to either TF or ODOM as  `robot_agent_12/odom` and child `robot_agent_12/baselink`.

- `multi_detection.launch.py`: Launch file for the new node.

## Odom

These has to be valid for both humanoid and pallet trucks

- world : is the origin (0,0,0)
- odom : is in the initial pose of robot
- base_link: tracks the movement of the robots relative to the odom

![Coordinate](/resources/coordsystems_img.png)
