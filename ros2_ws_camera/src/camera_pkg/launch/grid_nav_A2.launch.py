import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'camera_pkg'
    pkg_share_dir = get_package_share_directory(pkg_name)

    # Config file might be needed for some nodes
    default_config_path = os.path.join(pkg_share_dir, 'config', 'grid_config.yaml')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Full path to configuration YAML file'
    )

    # --- Node Definitions ---

    # 1. Grid Navigator (Assignment 2 - BFS)
    grid_navigator_A2_node = Node( # Changed variable name for clarity
            package=pkg_name,
            executable='grid_navigator_A2', # Use the new executable name from setup.py
            name='grid_commander_node', # Keep same node name if params/logic expect it
            output='screen',
            emulate_tty=True
            # parameters=[LaunchConfiguration('config_file')] # Uncomment if needed
        )

    # 2. Lane Centering Node
    lane_centering_node = Node(
            package=pkg_name,
            executable='lane_centering_node', # Check setup.py
            name='lane_centering_node',
            output='screen',
            emulate_tty=True
            # parameters=[LaunchConfiguration('config_file')] # Uncomment if needed
        )

    # 3. Turn Controller Node
    turn_controller_node = Node(
            package=pkg_name,
            executable='turn_controller', # Check setup.py
            name='turtlebot_turn_controller', # Check its __init__
            output='screen',
            emulate_tty=True
            # parameters=[LaunchConfiguration('config_file')] # Uncomment if needed
        )

    # 4. Edge Detection Node (NEW)
    edge_detection_node = Node(
            package=pkg_name,
            executable='edge_detection', # Check setup.py
            name='camera_subscriber', # Check its __init__
            output='screen',
            emulate_tty=True
            # parameters=[LaunchConfiguration('config_file')] # Uncomment if needed
        )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    ld.add_action(config_file_arg) # Keep if any node uses the config file

    # Add all nodes
    ld.add_action(grid_navigator_A2_node) # Use the A2 node
    ld.add_action(lane_centering_node)
    ld.add_action(turn_controller_node)
    ld.add_action(edge_detection_node)

    return ld

# ros2 launch camera_pkg grid_nav_A2.launch.py