# aruco_localization package

This is the aruco_localization package, which runs a node that takes input from `ID/camera_info` and `ID/image_raw`, with one camera per node. It then publishes the TF of the detected ArUco marker, mimicking Volvo's logic for tracking robot positions and generating navigation routes.

A launch file is provided to run multiple nodes, with each node dedicated to a single camera.

- List the cameras you want detection for in the control.sh file.

- If two cameras are pointed at the same ArUco code, the system does not alternate between them, but instead shows the midpoint between the two detections.

- The child link is named based on the camera. For example, if camera 164 detects an ArUco marker with ID 12, the child link is named `camera_164_marker_12`.

- Inside the aruco_node, a callback logs the relative position of the newly created `aruco_link` from the `base_link`.

- Two new message types are added in case you want to publish these aruco_poses.

- Multiple cameras can be linked to the same ArUco code simultaneously.

- The OpenCV (cv2) library is used for ArUco detection and pose estimation. The relative pose from the camera to the ArUco marker is calculated and published to TF, where the parent is the camera and the child is the `aruco_marker_id`.

- `aruco_localization/aruco_localization/aruco_detection_node.py`: Handles ArUco detection and publishes the TF between the camera and marker.

- `aruco_localization/aruco_localization/aruco_pose_pub.py`: A new node that listens to all transforms from the camera to aruco_12_marker and outputs the transform to either TF or ODOM.

- `aruco_localization/launch/multi_detection.launch.py`: Launch file for the new node.

- An example of how you run the package is: `ros2 launch aruco_localization multi_detection.launch.py camera_ids:="[163, 164, 165]" use_sim_time:=true publish_to_odom:=true`

  - camera_ids: List of camera ids, each element gets a detection_node spawned.
  - use_sim_time: Whether to use simulated time
  - publish_to_odom: Default set to True, When True it publish the ArUco pose via Odom. When False, publish the ArUco pose to TF.
