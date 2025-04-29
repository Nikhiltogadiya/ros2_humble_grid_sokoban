import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'camera_pkg'
    # pkg_share_dir = get_package_share_directory(pkg_name)

    # # --- Declare Launch Argument for the Sokoban Map File ---
    # # Allows overriding the default map path via command line if needed
    # default_map_rel_path = 'maps/map_sk.csv' # Relative path within install/share/camera_pkg
    # sokoban_map_arg = DeclareLaunchArgument(
    #     'sokoban_map_csv',
    #     # Resolve the path relative to the package share directory
    #     default_value=os.path.join(pkg_share_dir, default_map_rel_path),
    #     description='Full path to the Sokoban map CSV file'
    # )
    # # -------------------------------------------------------

    # --- Node Definitions ---

    # 1. Sokoban Navigator Node (Assignment 3)
    sokoban_navigator_node = Node(
            package=pkg_name,
            # Check setup.py for the exact executable name you defined!
            executable='sokoban_navigator',
            name='sokoban_navigator_node', # The name this node uses
            output='screen',
            emulate_tty=True,
            # parameters=[{ # Pass parameters as a dictionary
            #     # Use the launch argument for the map path
            #     'sokoban_map_csv': LaunchConfiguration('sokoban_map_csv'),
            #     # Add other parameters needed by sokoban_navigator here if any
            #     # 'max_solver_depth': 100 # Example if needed
            # }]
        )

    # 2. Lane Centering Node (Reused)
    lane_centering_node = Node(
            package=pkg_name,
            executable='lane_centering_node', # Check setup.py
            name='lane_centering_node',
            output='screen',
            emulate_tty=True
        )

    # 3. Turn Controller Node (Reused)
    turn_controller_node = Node(
            package=pkg_name,
            executable='turn_controller', # Check setup.py
            name='turtlebot_turn_controller', # Check its __init__
            output='screen',
            emulate_tty=True
        )

    # 4. Edge Detection Node (Reused)
    edge_detection_node = Node(
            package=pkg_name,
            executable='edge_detection', # Check setup.py
            name='camera_subscriber', # Check its __init__
            output='screen',
            emulate_tty=True
        )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared map file argument
    # ld.add_action(sokoban_map_arg)

    # Add all the nodes to be launched
    ld.add_action(sokoban_navigator_node)
    ld.add_action(lane_centering_node)
    ld.add_action(turn_controller_node)
    ld.add_action(edge_detection_node)

    return ld

# ros2 launch camera_pkg sokoban_nav.launch.py