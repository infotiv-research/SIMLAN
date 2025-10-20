from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    log_level = LaunchConfiguration("log_level").perform(context)

    camera_ids_str = LaunchConfiguration("camera_ids").perform(context)
    camera_ids = [id for id in camera_ids_str.split(" ")]
    
    point_start_str = LaunchConfiguration("point_start").perform(context)
    point_start = [float(value) for value in point_start_str.split(" ")]
    
    point_end_str = LaunchConfiguration("point_end").perform(context)
    point_end = [float(value) for value in point_end_str.split(" ")]
    
    resolution = float(LaunchConfiguration("resolution").perform(context))
    update_rate = int(LaunchConfiguration("update_rate").perform(context))
    input_img = LaunchConfiguration("input_img").perform(context)
    
    grid_x = int(abs(point_end[0] - point_start[0]) / resolution)
    grid_y = int(abs(point_end[1] - point_start[1]) / resolution)
    grid_size = "{grid_x} {grid_y}".format(grid_x=grid_x, grid_y=grid_y)
    
    projection_nodes = []
    for camera_id in camera_ids:
        projection_nodes.append(
            Node(
                package="camera_bird_eye_view",
                executable="camera_bird_eye_view_publisher",
                parameters=[
                    {"camera_id":camera_id},
                    {"point_start":point_start_str},
                    {"point_end":point_end_str},
                    {"resolution":resolution},
                    {"input_img":input_img},
                ],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level
                ]
            )
        )


    camera_stitch_node = Node(
            package="camera_bird_eye_view",
            executable="camera_bird_eye_view_stitcher",
            parameters=[
                {"camera_ids": camera_ids_str},
                {"update_rate": update_rate},
                {"grid_size": grid_size}
            ],
            output="screen",
            arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level
                ]
        )
    
    return projection_nodes + [camera_stitch_node]


def generate_launch_description():

    # Launch arguments
    camera_ids = LaunchConfiguration("camera_ids", default="163 164")
    camera_ids_arg = DeclareLaunchArgument("camera_ids", default_value=camera_ids)

    point_start = LaunchConfiguration("point_start", default="-4 -8")
    point_start_arg = DeclareLaunchArgument("point_start", default_value=point_start)
    
    point_end = LaunchConfiguration("point_end", default="32 4")
    point_end_arg = DeclareLaunchArgument("point_end", default_value=point_end)

    resolution = LaunchConfiguration("resolution", default=0.02)
    resolution_arg = DeclareLaunchArgument("resolution", default_value=resolution)

    update_rate = LaunchConfiguration("update_rate", default=30)
    update_rate_arg = DeclareLaunchArgument("update_rate", default_value=update_rate)

    input_img = LaunchConfiguration("input_img", default="image_raw")
    input_img_arg = DeclareLaunchArgument("input_img", default_value=input_img)

    

    # Launch description and nodes
    ld = LaunchDescription()   

    ld.add_action(camera_ids_arg )
    ld.add_action(point_start_arg )
    ld.add_action(point_end_arg )
    ld.add_action(resolution_arg )
    ld.add_action(update_rate_arg )
    ld.add_action(input_img_arg )

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
