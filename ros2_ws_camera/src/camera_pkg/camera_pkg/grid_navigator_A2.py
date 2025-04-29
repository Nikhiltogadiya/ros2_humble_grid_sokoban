#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import math
import time
import traceback
import sys # For sys.exit
import collections

# Helper function to normalize angles to [-pi, pi]
def normalize_angle(angle):
    """ Normalize an angle to the range [-pi, pi]. """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class GridCommanderNode(Node):
    # --- States ---
    IDLE = 0
    CALCULATING_NEXT_MOVE = 1
    AWAITING_TURN_FEEDBACK = 2
    AWAITING_MOVE_FEEDBACK = 3
    ARRIVED_AT_SQUARE = 4
    FINISHED = 5
    FAILED = 6

    # --- Orientations (Radians) ---
    EAST = 0.0       # Moving towards +col
    NORTH = math.pi / 2.0 # Moving towards +row
    WEST = math.pi      # Moving towards -col
    SOUTH = -math.pi / 2.0 # Moving towards -row


    def __init__(self):
        super().__init__('grid_commander_node')
        self.grid_map = [
            [0, 0, 1, 1, 1],  # Row 0
            [0, 0, 1, 0, 0],  # Row 1
            [0, 0, 1, 0, 0],  # Row 2
            [0, 0, 0, 0, 0],  # Row 3
            [0, 0, 0, 0, 0]   # Row 4
        ]
        self.rows = len(self.grid_map)
        self.cols = len(self.grid_map[0]) if self.rows > 0 else 0
        # Calculate total number of free squares to visit
        self.total_free_squares = sum(row.count(0) for row in self.grid_map)
        self.get_logger().info(f"Grid Map Initialized: {self.rows}x{self.cols}, Target Free Squares: {self.total_free_squares}")
        # --- Parameters ---
        # Grid Structure
        self.declare_parameter('tape_thickness_m', 0.01) # e.g., 1cm=0.01 tape thicknedd
        # Define a default 5x5 structure; best loaded from YAML
        # default_dims = [
        #     # Row 0
        # [
        #     {'length': 0.22, 'width': 0.21},  # Square (0,0)
        #     {'length': 0.23, 'width': 0.23},  # Square (0,1)
        #     {'length': 0.235, 'width': 0.235},  # Square (0,2)
        #     {'length': 0.25, 'width': 0.25},  # Square (0,3)
        #     {'length': 0.25, 'width': 0.25},  # Square (0,4)
        # ],
        # # Row 1
        # [
        #     {'length': 0.217, 'width': 0.21},  # Square (1,0)
        #     {'length': 0.215, 'width': 0.24},  # Square (1,1)
        #     {'length': 0.22, 'width': 0.23},  # Square (1,2)
        #     {'length': 0.22, 'width': 0.225},  # Square (1,3)
        #     {'length': 0.215, 'width': 0.214},  # Square (1,4)
        # ],
        # # Row 2
        # [
        #     {'length': 0.217, 'width': 0.207},  # Square (2,0)
        #     {'length': 0.217, 'width': 0.235},  # Square (2,1)
        #     {'length': 0.217, 'width': 0.231},  # Square (2,2)
        #     {'length': 0.222, 'width': 0.226},  # Square (2,3)
        #     {'length': 0.224, 'width': 0.214},  # Square (2,4)
        # ],
        # # Row 3
        # [
        #     {'length': 0.203, 'width': 0.203},  # Square (3,0)
        #     {'length': 0.216, 'width': 0.233},  # Square (3,1)
        #     {'length': 0.214, 'width': 0.233},  # Square (3,2)
        #     {'length': 0.227, 'width': 0.226},  # Square (3,3)
        #     {'length': 0.225, 'width': 0.211},  # Square (3,4)
        # ],
        # # Row 4
        # [
        #     {'length': 0.257, 'width': 0.257},  # Square (4,0)
        #     {'length': 0.252, 'width': 0.253},  # Square (4,1)
        #     {'length': 0.252, 'width': 0.252},  # Square (4,2)
        #     {'length': 0.245, 'width': 0.245},  # Square (4,3)
        #     {'length': 0.237, 'width': 0.237},  # Square (4,4)
        # ],
            
            # [{'length': 0.2, 'width': 0.2}] * 5,
            # [{'length': 0.2, 'width': 0.2}] * 5,
            # [{'length': 0.2, 'width': 0.2}] * 5,
            # [{'length': 0.2, 'width': 0.2}] * 5,
            # [{'length': 0.2, 'width': 0.2}] * 5,
        # ]
        # self.declare_parameter('square_dimensions', []) # or default_dims
        # self.declare_parameter('square_dimensions', default_dims) # or default_dims
        self.declare_parameter('start_row', 0)
        self.declare_parameter('start_col', 0)
        self.declare_parameter('start_orientation_rad', self.EAST) # Default start facing East

        # Topics
        self.declare_parameter('move_forward_topic', '/move_forward_distance')
        self.declare_parameter('distance_status_topic', '/distance_status')
        self.declare_parameter('turn_command_topic', '/turn_command')
        self.declare_parameter('turn_status_topic', '/turn_status')

        # Behaviour
        self.declare_parameter('control_frequency', 2.0) # Rate for state checks Hz
        self.declare_parameter('angle_tolerance_rad', math.radians(5.0)) # Tolerance for deciding if turn is needed
        self.declare_parameter('wait_time_at_square_sec', 0.5) # Pause after arrival confirmation


        # Get parameter values
        self.tape_thickness = self.get_parameter('tape_thickness_m').value
        # self.square_dims = self.get_parameter('square_dimensions').value
        self.start_pos = (0, 0) # From your script
        self.start_row = self.start_pos[0]
        self.start_col = self.start_pos[1]
        self.current_orientation_rad = self.get_parameter('start_orientation_rad').value
        
        # --- HARDCODE the square dimensions data ---
        grid_data = [
            # Row 0
            [ {'length': 0.22, 'width': 0.21}, {'length': 0.23, 'width': 0.23}, {'length': 0.235, 'width': 0.235}, {'length': 0.25, 'width': 0.25}, {'length': 0.25, 'width': 0.25} ],
            # Row 1
            [ {'length': 0.217, 'width': 0.21}, {'length': 0.215, 'width': 0.24}, {'length': 0.22, 'width': 0.23}, {'length': 0.22, 'width': 0.225}, {'length': 0.215, 'width': 0.214} ],
            # Row 2
            [ {'length': 0.217, 'width': 0.207}, {'length': 0.217, 'width': 0.235}, {'length': 0.217, 'width': 0.231}, {'length': 0.222, 'width': 0.226}, {'length': 0.224, 'width': 0.214} ],
            # Row 3
            [ {'length': 0.203, 'width': 0.203}, {'length': 0.216, 'width': 0.233}, {'length': 0.214, 'width': 0.233}, {'length': 0.227, 'width': 0.226}, {'length': 0.225, 'width': 0.211} ],
            # Row 4
            [ {'length': 0.257, 'width': 0.257}, {'length': 0.252, 'width': 0.253}, {'length': 0.252, 'width': 0.252}, {'length': 0.245, 'width': 0.245}, {'length': 0.237, 'width': 0.237} ]
        ]
        # --- Assign the hardcoded data ---
        self.square_dims = grid_data
        
        move_forward_topic = self.get_parameter('move_forward_topic').value
        distance_status_topic = self.get_parameter('distance_status_topic').value
        turn_command_topic = self.get_parameter('turn_command_topic').value
        turn_status_topic = self.get_parameter('turn_status_topic').value

        control_period = 1.0 / self.get_parameter('control_frequency').value
        self.angle_tolerance = self.get_parameter('angle_tolerance_rad').value
        self.wait_time = self.get_parameter('wait_time_at_square_sec').value

        # Validate grid dimensions from parameters
        if not isinstance(self.square_dims, list) or len(self.square_dims) != 5:
            self.get_logger().fatal("Parameter 'square_dimensions' must be a list of 5 rows.")
            sys.exit(1)
        for r, row_data in enumerate(self.square_dims):
            if not isinstance(row_data, list) or len(row_data) != 5:
                 self.get_logger().fatal(f"Row {r} in 'square_dimensions' must be a list of 5 squares.")
                 sys.exit(1)
            for c, square_data in enumerate(row_data):
                 if not isinstance(square_data, dict) or 'length' not in square_data or 'width' not in square_data:
                     self.get_logger().fatal(f"Square ({r},{c}) must be a dict with 'length' and 'width'.")
                     sys.exit(1)
        self.rows = 5
        self.cols = 5
        self.total_squares = self.rows * self.cols

        # --- Publishers ---
        self.move_pub = self.create_publisher(Float32, move_forward_topic, 10)
        self.turn_pub = self.create_publisher(String, turn_command_topic, 10)

        # --- Subscribers ---
        self.dist_sub = self.create_subscription(
            String, distance_status_topic, self.distance_status_callback, 10)
        self.turn_sub = self.create_subscription(
            String, turn_status_topic, self.turn_status_callback, 10)

        # --- State Variables ---
        self.current_state = self.IDLE
        self.target_row = 0
        self.target_col = 0
        self.current_grid_row = self.start_row
        self.current_grid_col = self.start_col
        self.visited_squares = set() # Keep track of visited (row, col) tuples
        self.last_action_time = self.get_clock().now() # To manage waits

        self.action_in_progress = False # Flag to prevent sending new commands while waiting

        self.get_logger().info(f"Grid Map Initialized: {self.rows}x{self.cols}, Target Free Squares: {self.total_free_squares}, Start: {self.start_pos}")
        
        # --- Timer for Control Loop ---
        # This timer mainly drives state transitions when not waiting for external feedback
        self.control_timer = self.create_timer(control_period, self.control_loop_callback)

        self.get_logger().info(
            f"Grid Commander Initialized: {self.rows}x{self.cols} grid."
            f" Start: ({self.start_row},{self.start_col}), "
            f"Orientation: {math.degrees(self.current_orientation_rad):.1f} deg."
        )
        self.get_logger().info(f" Tape thickness: {self.tape_thickness:.3f}m")
        self.get_logger().info(f" Pub move_cmd: '{move_forward_topic}', turn_cmd: '{turn_command_topic}'")
        self.get_logger().info(f" Sub status_dist: '{distance_status_topic}', status_turn: '{turn_status_topic}'")
        self.get_logger().info("Starting in IDLE state.")

        # Start navigation after a brief pause
        # self.create_timer(3.0, self.initialize_navigation, oneshot=True)
        self.initialization_timer = self.create_timer(3.0, self.initialize_navigation)

    def _get_neighbors(self, cell):
        """ Find valid, free neighbors for BFS """
        # grid access via self.grid_map
        # rows/cols via self.rows, self.cols
        r, c = cell
        for dr, dc in [(-1, 0), (0, 1), (1, 0), (0, -1)]: # N, E, S, W
            nr, nc = r + dr, c + dc
            # Check bounds and if the neighbor square is free (0)
            if 0 <= nr < self.rows and 0 <= nc < self.cols and self.grid_map[nr][nc] == 0:
                yield (nr, nc)
    
    def _bfs_parents(self, start_cell):
        """ Performs BFS returning parent mapping and distance dict. """
        queue = collections.deque([start_cell])
        parent = {start_cell: None}
        dist = {start_cell: 0}
        while queue:
            cell = queue.popleft()
            # Use the class method _get_neighbors
            for nbr in self._get_neighbors(cell):
                if nbr not in parent:
                    parent[nbr] = cell
                    dist[nbr] = dist[cell] + 1
                    queue.append(nbr)
        return parent, dist
    
    def _reconstruct_path(self, parent, start, end):
         """ Reconstructs path from start to end using parent map """
         # Handle cases where end is not reachable from start
         if end not in parent:
             return [] # Or raise error
         path = [end]
         # Check if path[-1] exists in parent before accessing
         while path[-1] is not None and path[-1] != start:
             parent_cell = parent.get(path[-1])
             if parent_cell is None: # Reached start or dead end
                  # If path[-1] isn't the start, means end wasn't reachable from start
                  if path[-1] != start: return []
                  break
             path.append(parent_cell)
         return list(reversed(path))

    
    def initialize_navigation(self):
        """ Sets up the starting state """
        if self.current_state == self.IDLE:
             # Mark the starting square as visited
            if 0 <= self.start_row < self.rows and 0 <= self.start_col < self.cols:
                self.visited_squares.add((self.start_row, self.start_col))
                self.get_logger().info(f"Marked starting square ({self.start_row}, {self.start_col}) as visited.")
                self.log_grid_status() # Show initial state
                self.current_state = self.ARRIVED_AT_SQUARE # Start as if we just arrived
                self.last_action_time = self.get_clock().now()
            else:
                self.get_logger().error(f"Start position ({self.start_row}, {self.start_col}) is outside the 5x5 grid! Navigation FAILED.")
                self.current_state = self.FAILED
                
        # ---- ADD THIS LINE ----
        # Cancel the timer so this function doesn't run again automatically
        if self.initialization_timer: # Optional check if timer exists
            self.initialization_timer.cancel()


    def get_square_dims(self, r, c):
        """ Safely get dimensions of a square """
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return self.square_dims[r][c]
        else:
            self.get_logger().error(f"Attempted to access invalid square ({r},{c}) dimensions!")
            return None

    # --- Status Callbacks ---
    def distance_status_callback(self, msg: String):
        if msg.data.lower() == "finished":
            self.get_logger().debug("Received 'finished' status for distance.")
            if self.current_state == self.AWAITING_MOVE_FEEDBACK:
                self.get_logger().info("Move action completed.")
                self.action_in_progress = False
                # Update internal grid position based on the target we were moving towards
                self.current_grid_row = self.target_row
                self.current_grid_col = self.target_col
                self.visited_squares.add((self.current_grid_row, self.current_grid_col))
                self.get_logger().info(f"Arrived at square ({self.current_grid_row}, {self.current_grid_col}).")
                self.current_state = self.ARRIVED_AT_SQUARE
                self.last_action_time = self.get_clock().now() # Record arrival time for waiting
            else:
                 self.get_logger().warn(f"Received 'finished' distance status while not in AWAITING_MOVE_FEEDBACK state (State: {self.current_state}). Ignoring.")
        # Add handling for other status messages if needed (e.g., "error")

    def turn_status_callback(self, msg: String):
        if msg.data.lower() == "finished":
            self.get_logger().debug("Received 'finished' status for turn.")
            if self.current_state == self.AWAITING_TURN_FEEDBACK:
                self.get_logger().info("Turn action completed.")
                self.action_in_progress = False
                # Orientation is updated when the command is issued.
                # Now proceed to move
                self.current_state = self.CALCULATING_NEXT_MOVE # Re-calculate to issue move command
            else:
                self.get_logger().warn(f"Received 'finished' turn status while not in AWAITING_TURN_FEEDBACK state (State: {self.current_state}). Ignoring.")
         # Add handling for other status messages if needed

    # --- Action Execution ---
    def send_turn_command(self, direction: str):
        """ Sends 'left' or 'right' turn command """
        if self.action_in_progress:
            self.get_logger().warn("Action already in progress, cannot send turn command.")
            return False
        if direction.lower() not in ["left", "right"]:
             self.get_logger().error(f"Invalid turn direction: {direction}")
             return False

        cmd = String()
        cmd.data = direction.lower()
        self.turn_pub.publish(cmd)
        self.action_in_progress = True
        self.current_state = self.AWAITING_TURN_FEEDBACK
        self.get_logger().info(f"Published turn command: '{cmd.data}'. Waiting for status...")
        return True

    def send_move_command(self, distance: float):
        """ Sends move forward command """
        if self.action_in_progress:
            self.get_logger().warn("Action already in progress, cannot send move command.")
            return False
        if distance <= 0:
             self.get_logger().warn(f"Attempting to move zero or negative distance ({distance:.3f}). Skipping move.")
             # If skipping move, consider the target reached immediately
             self.current_grid_row = self.target_row
             self.current_grid_col = self.target_col
             self.visited_squares.add((self.current_grid_row, self.current_grid_col))
             self.current_state = self.ARRIVED_AT_SQUARE
             self.last_action_time = self.get_clock().now()
             return True # Technically successful in not needing to move

        cmd = Float32()
        cmd.data = distance
        self.move_pub.publish(cmd)
        self.action_in_progress = True
        self.current_state = self.AWAITING_MOVE_FEEDBACK
        self.get_logger().info(f"Published move command: {cmd.data:.3f}m. Waiting for status...")
        return True

    # --- Grid Logic ---
    # def calculate_next_target_square(self):
    #     """ Determines the next square to visit (row-by-row). Returns (row, col) or None. """
    #     for r in range(self.rows):
    #         for c in range(self.cols):
    #             if (r, c) not in self.visited_squares:
    #                 return r, c
    #     return None
    def calculate_next_target_square(self):
        """
        Determines the next adjacent square to move towards the nearest
        unvisited free square using BFS.
        Returns (row, col) tuple for the next step, or None if finished.
        """
        current_pos = (self.current_grid_row, self.current_grid_col)

        # 1. Find all free squares and the ones not yet visited
        free_cells = set()
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid_map[r][c] == 0:
                    free_cells.add((r, c))

        unvisited_free_cells = free_cells - self.visited_squares

        # 2. Check if there are any unvisited free cells left
        if not unvisited_free_cells:
            self.get_logger().info("All reachable free squares have been visited.")
            return None # Finished

        # 3. Run BFS from the current position to find distances to all reachable cells
        parent_map, distance_map = self._bfs_parents(current_pos)

        # 4. Find the *nearest* unvisited free cell
        nearest_target = None
        min_dist = float('inf')

        for cell in unvisited_free_cells:
            dist = distance_map.get(cell, float('inf')) # Get distance from BFS result
            if dist < min_dist:
                min_dist = dist
                nearest_target = cell

        # 5. Check if any target was found (might be unreachable if grid is disconnected)
        if nearest_target is None:
            self.get_logger().warn(f"Could not find a reachable path to any remaining unvisited free cells from {current_pos}.")
            return None # Cannot reach remaining cells

        # 6. Reconstruct the path from current position to the NEAREST target
        path_segment = self._reconstruct_path(parent_map, current_pos, nearest_target)

        # 7. Determine the NEXT STEP along that path
        if path_segment and len(path_segment) > 1:
            next_step = path_segment[1] # The second element is the first step to take
            self.get_logger().info(f"Nearest unvisited free cell: {nearest_target} at dist {min_dist}. Next step: {next_step}")
            # Check if next step is blocked (shouldn't be if BFS worked, but safety check)
            if self.grid_map[next_step[0]][next_step[1]] == 1:
                 self.get_logger().error(f"BFS suggested moving to a blocked cell {next_step}! Check BFS logic.")
                 return None # Avoid moving to blocked cell
            return next_step
        elif path_segment and len(path_segment) == 1 and path_segment[0] == current_pos:
            # This can happen if the nearest unvisited is the current square, which shouldn't occur
            # if the visited set is updated correctly before calling this function.
             self.get_logger().warn(f"Nearest unvisited free cell {nearest_target} is current position? Check visited set logic.")
             # Mark current as visited if somehow it wasn't, and try again by returning None (will re-call)
             self.visited_squares.add(current_pos)
             return None
        else:
            # Path segment empty or only contains current position, meaning target is unreachable or is current pos
            self.get_logger().error(f"Could not reconstruct path to nearest target {nearest_target} from {current_pos}, or path is invalid.")
            return None # No valid next step found

    def calculate_move_details(self, target_r, target_c):
        """ Calculates required orientation, turn direction, and move distance. """
        delta_r = target_r - self.current_grid_row
        delta_c = target_c - self.current_grid_col

        # --- Determine Target Orientation ---
        target_orientation_rad = self.current_orientation_rad # Default: stay same
        if delta_r > 0: target_orientation_rad = self.NORTH
        elif delta_r < 0: target_orientation_rad = self.SOUTH
        elif delta_c > 0: target_orientation_rad = self.EAST
        elif delta_c < 0: target_orientation_rad = self.WEST
        # Else (delta_r == 0 and delta_c == 0): Target is current square, orientation doesn't matter

        # --- Determine Turn ---
        turn_direction = None
        angle_diff = normalize_angle(target_orientation_rad - self.current_orientation_rad)

        if abs(angle_diff) > self.angle_tolerance:
             # Determine shortest turn direction (left is positive angle diff in ROS standard)
             if angle_diff > 0:
                 turn_direction = "right" # Counter-clockwise 'left'
             else:
                 turn_direction = "left" # Clockwise 'right'

        # --- Determine Distance ---
        distance = 0.0
        current_dims = self.get_square_dims(self.current_grid_row, self.current_grid_col)
        target_dims = self.get_square_dims(target_r, target_c)

        if not current_dims or not target_dims:
            self.get_logger().error("Could not get square dimensions for distance calculation.")
            return None, None, 0.0 # Indicate error

        if delta_r != 0: # Moving vertically
             # Use length dimension (along the row direction)
             distance = (current_dims['length'] / 2.0) + self.tape_thickness + (target_dims['length'] / 2.0)
        elif delta_c != 0: # Moving horizontally
             # Use width dimension (along the col direction)
             distance = (current_dims['width'] / 2.0) + self.tape_thickness + (target_dims['width'] / 2.0)
        # else: distance remains 0 if target is current square

        return target_orientation_rad, turn_direction, distance, angle_diff

    def log_grid_status(self):
        """ Prints the current state of the grid, showing obstacles, visited free squares, robot position, and current square dimensions. """
        # --- Input Checks ---
        if not hasattr(self, 'grid_map') or not self.grid_map:
            self.get_logger().error("log_grid_status: grid_map not initialized!")
            return
        if not hasattr(self, 'square_dims') or not self.square_dims:
            self.get_logger().error("log_grid_status: square_dims not initialized!")
            return
        if not hasattr(self, 'visited_squares'):
            self.get_logger().error("log_grid_status: visited_squares not initialized!")
            return
        if not hasattr(self, 'total_free_squares'):
            try:
                self.total_free_squares = sum(row.count(0) for row in self.grid_map)
                self.get_logger().warn("log_grid_status: Recalculated missing total_free_squares.")
            except Exception:
                self.get_logger().error("log_grid_status: total_free_squares not initialized and cannot calculate!")
                return
        # --- End Input Checks ---

        log_str = "\n----- Grid Status (Assignment 2) -----\n"
        log_str += " R = Robot | . = Free | # = Blocked | X = Visited Free\n"
        separator = "-" * (self.cols * 4 + 1) + "\n"

        log_str += separator
        for r in range(self.rows):
            row_str = "| "
            for c in range(self.cols):
                char_to_display = '?'
                if r == self.current_grid_row and c == self.current_grid_col:
                    char_to_display = "R"
                elif self.grid_map[r][c] == 1: # Blocked
                    char_to_display = "#"
                elif (r, c) in self.visited_squares: # Visited Free
                    char_to_display = "X"
                else: # Free, Unvisited
                    char_to_display = "."
                row_str += f"{char_to_display} | "
            log_str += row_str + "\n"

        log_str += separator
        log_str += f" Visited Free: {len(self.visited_squares)} / {self.total_free_squares}\n"

        # --- ADDED: Display Current Square Dimensions ---
        current_r, current_c = self.current_grid_row, self.current_grid_col
        # Use the get_square_dims helper for safety
        current_dims = self.get_square_dims(current_r, current_c)
        if current_dims:
            log_str += f" Current Square ({current_r},{current_c}) Dims: L={current_dims['length']:.3f}m, W={current_dims['width']:.3f}m\n"
        else:
            # Check if current square is blocked (might not have dims defined)
            is_blocked = False
            if 0 <= current_r < self.rows and 0 <= current_c < self.cols:
                is_blocked = (self.grid_map[current_r][current_c] == 1)

            if is_blocked:
                log_str += f" Current Square ({current_r},{current_c}) Dims: N/A (Blocked Square)\n"
            else:
                log_str += f" Current Square ({current_r},{current_c}) Dims: Error retrieving!\n"
        # --- END ADDED ---

        log_str += f" Current Pos: ({current_r},{current_c})\n"
        log_str += "-------------------------------------\n"
        self.get_logger().info(log_str)

    # --- Main Control Loop ---
    def control_loop_callback(self):
        """ Main state machine logic, driven by timer and status callbacks. """

        if self.current_state in [self.FINISHED, self.FAILED]:
            return # Do nothing if finished or failed

        # Only proceed if no external action is currently awaited
        if self.action_in_progress:
             # self.get_logger().debug(f"Action in progress, waiting... (State: {self.current_state})", throttle_duration_sec=2.0)
             return

        # --- State: ARRIVED_AT_SQUARE ---
        if self.current_state == self.ARRIVED_AT_SQUARE:
            # Wait for the specified time before calculating next move
            elapsed_time = (self.get_clock().now() - self.last_action_time).nanoseconds / 1e9
            if elapsed_time >= self.wait_time:
                self.log_grid_status() # Log status after waiting
                self.current_state = self.CALCULATING_NEXT_MOVE
            else:
                return # Still waiting

        # --- State: CALCULATING_NEXT_MOVE ---
        if self.current_state == self.CALCULATING_NEXT_MOVE:
            self.get_logger().debug("State: CALCULATING_NEXT_MOVE")

            next_target = self.calculate_next_target_square()
            if next_target is None:
                self.get_logger().info("All squares visited! Navigation FINISHED.")
                self.current_state = self.FINISHED
                # Initiate shutdown after a short delay?
                self.create_timer(1.0, self.shutdown_sequence, oneshot=True)
                return
            
            self.target_row, self.target_col = next_target
            self.get_logger().info(f"Calculating move from ({self.current_grid_row},{self.current_grid_col}) to target ({self.target_row},{self.target_col})")

            # Prevent moving more than one square
            dr = abs(self.target_row - self.current_grid_row)
            dc = abs(self.target_col - self.current_grid_col)
            if not ((dr == 1 and dc == 0) or (dr == 0 and dc == 1)):
                 if dr == 0 and dc == 0: # Should not happen if visited set is correct
                    self.get_logger().warn("Target square is current square? Skipping move calculation.")
                    # This case might indicate logic error, but try to recover
                    self.visited_squares.add((self.current_grid_row, self.current_grid_col)) # Ensure marked visited
                    self.current_state = self.ARRIVED_AT_SQUARE # Treat as arrived
                    self.last_action_time = self.get_clock().now()
                    return
                 else:
                    self.get_logger().error(f"Attempting to move more than one square! From ({self.current_grid_row},{self.current_grid_col}) to ({self.target_row},{self.target_col}). Navigation FAILED.")
                    self.current_state = self.FAILED
                    return


            target_orientation, turn_dir, move_dist, angle_diff = self.calculate_move_details(self.target_row, self.target_col)

            if target_orientation is None: # Error during calculation
                self.get_logger().error("Failed to calculate move details. Navigation FAILED.")
                self.current_state = self.FAILED
                return

            # --- Decide Action: Turn or Move ---
            if turn_dir:
                self.get_logger().info(f"Turn required: Sending command '{turn_dir}' (based on angle_diff {angle_diff:.2f})")

                # --- CORRECTED Internal Orientation Update ---
                # Base the internal update on the SIGN of the original angle_diff,
                # NOT on the swapped turn_dir string.
                angle_change = 0.0
                if angle_diff > self.angle_tolerance: # Original condition for needing a CCW/Left turn
                    angle_change = math.pi / 2.0
                    self.get_logger().info("Updating internal orientation for intended LEFT turn (+pi/2)")
                elif angle_diff < -self.angle_tolerance: # Original condition for needing a CW/Right turn
                    angle_change = -math.pi / 2.0
                    self.get_logger().info("Updating internal orientation for intended RIGHT turn (-pi/2)")

                # Apply the update BEFORE sending command
                self.current_orientation_rad = normalize_angle(self.current_orientation_rad + angle_change)
                self.get_logger().info(f"Updated internal orientation to: {math.degrees(self.current_orientation_rad):.1f} deg")
                # --- END CORRECTED Update ---

                # Send command (using the swapped turn_dir string variable)
                self.send_turn_command(turn_dir)
                # State transition happens within send_turn_command
                
                # self.get_logger().info(f"Turn required: {turn_dir}")
                # # Update intended orientation *before* sending command
                # # This assumes the turn node will achieve the target 90 deg turn
                # angle_change = math.pi / 2.0 if turn_dir == "left" else -math.pi / 2.0
                # self.current_orientation_rad = normalize_angle(self.current_orientation_rad + angle_change)
                # self.get_logger().info(f"Updated internal orientation to: {math.degrees(self.current_orientation_rad):.1f} deg")
                # # Send command
                # self.send_turn_command(turn_dir)
                # # State transition happens within send_turn_command

            elif move_dist > 0.001 : # Check if significant move is needed (small tolerance)
                 self.get_logger().info(f"Move required: {move_dist:.3f}m")
                 self.send_move_command(move_dist)
                 # State transition happens within send_move_command
            else:
                 # No turn and no move needed? Should only happen if target is current square
                 self.get_logger().warn(f"No turn or move needed for target ({self.target_row},{self.target_col}). Marking arrived.")
                 self.visited_squares.add((self.target_row, self.target_col)) # Ensure marked visited
                 self.current_state = self.ARRIVED_AT_SQUARE # Treat as arrived
                 self.last_action_time = self.get_clock().now()


    def shutdown_sequence(self):
        self.get_logger().info("Initiating node shutdown...")
        # Stop the timer
        if self.control_timer:
            self.control_timer.cancel()
        # Optional: publish a final "idle" or "stop" command if your action nodes support it
        # ...
        #rclpy.shutdown() # Let the main finally block handle this
        # A more forceful exit if needed:
        sys.exit(0)

    def shutdown_hook(self):
        """ Called externally on shutdown. """
        self.get_logger().info("Shutdown hook called.")
        self.current_state = self.FAILED # Stop processing


def main(args=None):
    rclpy.init(args=args)
    grid_commander_node = None
    try:
        grid_commander_node = GridCommanderNode()
        rclpy.spin(grid_commander_node)
    except KeyboardInterrupt:
        print('Keyboard interrupt, shutting down.')
    except SystemExit:
        print('Shutdown initiated by node.')
    except Exception as e:
        if grid_commander_node:
            grid_commander_node.get_logger().fatal(f"Unhandled exception: {e}")
            traceback.print_exc()
        else:
            print(f"Unhandled exception before node init: {e}")
            traceback.print_exc()
    finally:
        # Ensure cleanup on exit
        if grid_commander_node:
            grid_commander_node.shutdown_hook()
            # Give a brief moment for last messages if needed
            # time.sleep(0.1)
            grid_commander_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 Shutdown complete.")

if __name__ == '__main__':
    main()