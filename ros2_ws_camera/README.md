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