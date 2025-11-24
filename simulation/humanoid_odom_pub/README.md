# Humanoid_odom_pub

When driving around with the humanoid it was noticed that the tfs the ros2 controller published did not cohere with the humanoids' position in gazebo. Since the humanoids purpose is only to simulate human behavior it was decided to not use the odom frame from the ros2 controllers but to get the odom frame from the gazebo pose instead. Therefore, the humanoid_odom_pub was created to get that pose and use it as the odom frame.

This is done by bridging the poses of the robot_agents and humanoids to the `namespace/pose` topic. Then a subscriber is created which listens to the given topic and publishes a transform between `namespace/odom` -> `namespace/base_link` depending on the pose of the robots. This works as the ground truth of the position.

## How to remove

If it is decided to remove this pkg in the future follow these steps:

1. got to simulation/humanoid_support_moveit_config/launch/launch_controllers.launch.py and delete the following:

   ```python
   odom_publisher = Node(
        package="humanoid_odom_pub",
        executable="humanoid_odom_pub",
        name="humanoid_odom_pub",
        output="screen",
        namespace=namespace,
        parameters=[
                    {"namespace": namespace},
                    {"initial_pose_x": initial_pose_x},
                    {"initial_pose_y": initial_pose_y},
                    {'use_sim_time': True}
                    ],
    )
    ...
    actions.append(odom_publisher)
   ```

1. The humanoids still need a dynamic transform from  humanoid_x/odom -> humanoid_x/base_link. Either implement the newly decided solution or revert the changes in the ros2_controller. To revert changes in the ros2_controller go to config_generation/scripts/generate_humanoid_control_yaml.py and change:

   ```python
   "enable_odom_tf": False,
   ```

   to:

   ```python
   "enable_odom_tf": True,
   ```

   Please note if the pose and orientation in both gazebo and rviz are the same after teleoping or doing navigation to not break the navigation stack for the humanoids.

1. Delete the simulation/humanoid_odom_pub pkg

1. Build the workspace
