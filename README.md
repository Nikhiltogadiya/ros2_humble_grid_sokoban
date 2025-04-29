# ros2_humble_grid_sokoban

# TurtleBot3 ROS2 Humble Workspace - Grid Navigation & Sokoban

This ROS2 workspace contains packages for controlling a TurtleBot3 robot to perform navigation tasks within a 2D grid environment using ROS2 Humble. It includes solutions for three assignments: basic grid traversal, traversal with obstacles, and a Sokoban puzzle.

**Core Functionality:**

* **Camera Processing:** Uses `edge_detection.py` to process camera images (`/image_raw` topic), detect lane lines using HSV color masking and line segment detection, and publish the results (`/detected_lane_data` topic).
* **Movement Control:**
    * `lane_centering_node.py`: Manages forward movement. It subscribes to `/move_forward_distance` commands and `/detected_lane_data`. It uses odometry (`/odom`) and lane data to move the robot a specified distance while attempting to stay centered between detected lane lines, publishing velocity commands to `/cmd_vel` and status updates to `/distance_status`. It includes alignment logic before and after moving.
    * `turn_controller.py`: Manages turning. It subscribes to `/turn_command` (`"left"`, `"right"`, `"uturn"`) and uses odometry (`/odom`) to execute 90-degree or 180-degree turns, publishing velocity commands to `/cmd_vel` and status updates to `/turn_status`.
* **High-Level Navigation:** Nodes orchestrate sequences of turns and forward movements to complete specific tasks.

## Workspace Structure

* **`ros2_ws_camera/`**: The root of the ROS2 workspace.
    * **`src/`**: Contains the source code for the ROS2 packages.
        * **`camera_pkg/`**: The primary package containing all nodes and logic.
            * **`camera_pkg/`**: Python module source files.
                * `edge_detection.py`: Detects lane lines from the camera feed.
                * `lane_centering_node.py`: Controls precise forward movement and centering.
                * `turn_controller.py`: Controls turning actions.
                * `grid_navigator.py`: Orchestrates Assignment 1 (full grid traversal).
                * `grid_navigator_A2.py`: Orchestrates Assignment 2 (traversal with blocked squares) using BFS.
                * `sokoban_navigator.py`: Orchestrates Assignment 3 (Sokoban puzzle) using A* search.
                * **`sokoban/`**: Sub-package containing Sokoban game logic, utilities, and the A* solver.
                    * `sk_game.py`: Defines the Sokoban game state, map loading (currently hardcoded), and basic move logic.
                    * `sk_solver.py`: Implements the A* search algorithm to find a Sokoban solution and provides visualization capabilities.
                    * `sk_solver_util.py`: Contains utility classes for the A* solver, including the heuristic calculation and node representation.
                    * `sk_utilities.py`: Defines enums for Tiles, Directions, and the Push action.
            * **`config/`**: Configuration files.
                * `grid_config.yaml`: Parameters for grid dimensions (though currently hardcoded in nodes) and topic names. *(Note: The grid dimensions seem hardcoded in the Python files `grid_navigator.py`, `grid_navigator_A2.py`, and `sokoban_navigator.py`, potentially overriding the YAML file)*.
            * **`launch/`**: Launch files to start the nodes for each assignment.
                * `grid_nav.launch.py`: Launches nodes for Assignment 1.
                * `grid_nav_A2.launch.py`: Launches nodes for Assignment 2.
                * `sokoban_nav.launch.py`: Launches nodes for Assignment 3.
            * **`maps/`**: Contains map data.
                * `map_sk.csv`: Map definition for the Sokoban puzzle (currently hardcoded in `sk_game.py`).
            * `package.xml`: Package manifest defining dependencies and metadata[cite: 1].
            * `setup.cfg`: Configuration for package installation[cite: 2].
            * `setup.py`: Defines how the package is built and installed, including node entry points.

## Assignments

### Assignment 1: Visit Each Square

* **Goal:** Move the robot to visit every square in a 5x5 grid.
* **Logic:** Uses `grid_navigator.py`, which follows a serpentine path (row by row, alternating direction) to visit all squares. It calculates the required turns and distances based on hardcoded square dimensions and tape thickness.
* **Fail Conditions:** Touching square boundary, moving two squares at once, moving outside the matrix.
* **Launch:** `ros2 launch camera_pkg grid_nav.launch.py`

### Assignment 2: Move with Blocked Squares

* **Goal:** Visit every *free* square in a grid containing blocked squares, using a predefined map.
* **Logic:** Uses `grid_navigator_A2.py`, which incorporates a hardcoded grid map representing obstacles. It employs a Breadth-First Search (BFS) algorithm to find the path to the nearest unvisited free square. Movement calculation uses hardcoded square dimensions.
* **Fail Conditions:** Moving two squares at once, stopping on a boundary, moving into a blocked square, moving outside the matrix.
* **Launch:** `ros2 launch camera_pkg grid_nav_A2.launch.py`

### Assignment 3: Sokoban (50%)

* **Goal:** Use the robot to push two boxes to predefined goal locations within a 2D matrix.
* **Logic:** Uses `sokoban_navigator.py` which orchestrates the task.
    * It initializes the Sokoban game state (`sk_game.py`) using a hardcoded map.
    * It runs an A* solver (`sk_solver.py`, `sk_solver_util.py`) to find a sequence of pushes to solve the puzzle.
    * It translates the high-level push plan into a sequence of relative robot movements (FORWARD, TURN_LEFT, TURN_RIGHT, UTURN).
    * It executes these relative moves by sending commands to `lane_centering_node.py` and `turn_controller.py`.
* **Fail Conditions:** Robot/box moves outside matrix, stopping on boundary, moving two fields, pushing two boxes, pulling a box, moving into a blocked field.
* **Launch:** `ros2 launch camera_pkg sokoban_nav.launch.py`

## Setup and Usage

1.  **Prerequisites:**
    * ROS2 Humble installed.
    * TurtleBot3 simulation environment (like Gazebo) or physical robot setup.
    * Dependencies listed in `package.xml` [cite: 1] (rclpy, sensor_msgs, cv_bridge, opencv-python, tf_transformations). Install them using `rosdep` or `pip`.

2.  **Build the Workspace:**
    * Navigate to the root of the workspace (`ros2_ws_camera`).
    * Run `colcon build --packages-select camera_pkg`.

3.  **Source the Workspace:**
    * In *every new terminal* you use for this workspace, run `source install/setup.bash`.

4.  **Launch an Assignment:**
    * Ensure your TurtleBot3 simulation (e.g., Gazebo) or the physical robot is running and publishing necessary topics (`/image_raw`, `/odom`).
    * Open a new terminal, source the workspace (step 3).
    * Run the corresponding launch file:
        * Assignment 1: `ros2 launch camera_pkg grid_nav.launch.py`
        * Assignment 2: `ros2 launch camera_pkg grid_nav_A2.launch.py`
        * Assignment 3: `ros2 launch camera_pkg sokoban_nav.launch.py`

## Modification and Extension

* **Parameters:** Many parameters like movement speeds, gains (Kp), tolerances, and topic names are defined in the Python node files (e.g., `lane_centering_node.py`) or launch files. Modify these values directly in the code or adjust the launch files/YAML configuration (`grid_config.yaml`, although note the hardcoding overrides mentioned earlier).
* **Grid Dimensions/Maps:**
    * For Assignments 1 & 2, the individual square dimensions are currently hardcoded within `grid_navigator.py` and `grid_navigator_A2.py`. You'll need to modify these Python lists directly to change the grid layout.
    * For Assignment 2, the obstacle map is hardcoded in `grid_navigator_A2.py`. Modify the `self.grid_map` list there.
    * For Assignment 3 (Sokoban), the map layout is hardcoded in `camera_pkg/sokoban/sk_game.py`. Modify the `hardcoded_map_data` list within the `__init__` method. You can use the `Tile` enum values from `sk_utilities.py` to define walls, floor, robot start, box starts, and goal locations. Remember that the numerical values in the map often represent combinations (e.g., Robot + Floor = 8 + 2 = 10).
* **Logic:** The core logic for navigation (serpentine, BFS, A*) and movement control resides in the respective Python files. You can modify these scripts to change algorithms or behaviors.
* **Adding Nodes:** Define new nodes in Python, add entry points in `setup.py`, and include them in the relevant launch files.