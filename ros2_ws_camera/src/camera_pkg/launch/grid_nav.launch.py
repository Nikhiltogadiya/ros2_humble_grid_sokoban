import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Find the folder where ROS installed your package files
    pkg_name = 'camera_pkg'
    pkg_share_dir = get_package_share_directory(pkg_name)

    # 2. Point to the NEW location of your config file WITHIN the installed folder
    #    Since setup.py puts it in a 'config' subfolder, we add '/config/' here.
    default_config_path = os.path.join(pkg_share_dir, 'config', 'grid_config.yaml')

    # # 3. Declare an argument so you could potentially use a different config file later
    # config_file_arg = DeclareLaunchArgument(
    #     'config_file',
    #     default_value=default_config_path,
    #     description='Full path to the grid commander configuration YAML file'
    # )

    # 4. Define the robot program (node) we want to start
    grid_commander_node = Node(
            package=pkg_name,           # Name of your package
            executable='grid_navigator', # The name from setup.py's 'entry_points'
            name='grid_commander_node', # The name the node uses internally (from your Python code)
            output='screen',            # Show messages in the terminal
            emulate_tty=True,           # Helps with printing
            # parameters=[LaunchConfiguration('config_file')] # Tell the node to use the config file!
        )
    
    # 2. Lane Centering Node (Handles forward movement)
    lane_centering_node = Node(
            package=pkg_name,
            executable='lane_centering_node', # Check this name against setup.py entry_points
            name='lane_centering_node', # Default node name unless specified otherwise
            output='screen',
            emulate_tty=True
            # Add parameters=[...] here if lane_centering_node uses the YAML file
        )
    
    # 3. Turn Controller Node (Handles turning)
    turn_controller_node = Node(
            package=pkg_name,
            executable='turn_controller', # Check this name against setup.py entry_points
            name='turtlebot_turn_controller', # Use the name from its super().__init__() call
            output='screen',
            emulate_tty=True
            # Add parameters=[...] here if turn_controller uses the YAML file
        )
    # 4. Edge Detection Node (NEW)
    edge_detection_node = Node(
            package=pkg_name,
            executable='edge_detection', # Check setup.py entry point name for edge_detection.py
            name='camera_subscriber', # Check class name CameraSubscriber's super().__init__()
            output='screen',
            emulate_tty=True
            # parameters=[LaunchConfiguration('config_file')] # Uncomment if it needs params from file
        )

    # 5. Create the launch description and add the node to it
    ld = LaunchDescription()

    ld.add_action(grid_commander_node) # Add your node
    ld.add_action(lane_centering_node) # Add your node
    ld.add_action(turn_controller_node) # Add your node
    ld.add_action(edge_detection_node) # Add your node

    # --- REMEMBER ---
    # You need other nodes running too! Like the one that actually controls
    # the motors based on commands from grid_navigator_2.
    # You would add more 'Node(...)' definitions here for those later.
    #
    # Example (replace with your actual motor node):
    # motor_node = Node(
    #      package='some_motor_package',
    #      executable='motor_driver',
    #      name='motor_driver_node'
    # )
    # ld.add_action(motor_node)
    # ---------------

    return ld

# command to launch
# ros2 launch camera_pkg grid_nav.launch.py