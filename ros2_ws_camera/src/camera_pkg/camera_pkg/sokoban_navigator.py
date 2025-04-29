#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import math
import time
import traceback
import sys
import numpy as np
from typing import List, Tuple, Optional

# --- Import Sokoban specific modules ---
try:
    from camera_pkg.sokoban.sk_solver import AStar
    from camera_pkg.sokoban.sk_utilities import RobotDirection, Push
    from camera_pkg.sokoban.sk_game import SokobanGame
except ImportError as e:
    print(f"Error importing Sokoban modules: {e}")
    print("Please ensure sk_solver.py, sk_utilities.py, and sk_game.py are in the 'camera_pkg/sokoban' directory")
    print("and that the __init__.py file exists in 'camera_pkg/sokoban'.")
    print("Also check that 'camera_pkg' is correctly listed in setup.py's packages.")
    sys.exit(1)

# --- Constants for Relative Moves ---
FORWARD = "FORWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
UTURN = "UTURN"
# --- End Constants ---

# Helper function
def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class SokobanNavigatorNode(Node):
    # --- States ---
    IDLE = 0
    SOLVING = 1
    EXECUTING_MOVE = 2
    AWAITING_TURN = 3
    AWAITING_MOVE = 4
    WAITING_AFTER_STEP = 5
    # Removed AWAITING_UTURN_PART2 state, using flags instead
    FINISHED = 6 # Renumbered
    FAILED = 7   # Renumbered

    # --- Orientations (Radians) ---
    EAST = 0.0
    NORTH = math.pi / 2.0
    WEST = math.pi
    SOUTH = -math.pi / 2.0

    ORIENTATION_TO_ROBOTDIR = {NORTH: RobotDirection.UP, SOUTH: RobotDirection.DOWN, WEST: RobotDirection.LEFT, EAST: RobotDirection.RIGHT}
    ROBOTDIR_TO_ORIENTATION = {v: k for k, v in ORIENTATION_TO_ROBOTDIR.items()}

    def __init__(self):
        super().__init__('sokoban_navigator_node')

        # --- Parameters ---
        self.declare_parameter('tape_thickness_m', 0.025)
        try:
            temp_game = SokobanGame()
            start_r, start_c = np.unravel_index(temp_game.robot_index, (temp_game.grid_rows, temp_game.grid_cols))
            del temp_game
        except Exception as e:
            self.get_logger().fatal(f"Failed to get start pos from SokobanGame: {e}")
            raise

        self.declare_parameter('start_row', int(start_r))
        self.declare_parameter('start_col', int(start_c))
        self.declare_parameter('start_orientation_rad', self.NORTH)
        self.declare_parameter('move_forward_topic', '/move_forward_distance')
        self.declare_parameter('distance_status_topic', '/distance_status')
        self.declare_parameter('turn_command_topic', '/turn_command')
        self.declare_parameter('turn_status_topic', '/turn_status')
        self.declare_parameter('control_frequency', 5.0)
        self.declare_parameter('wait_time_after_step_sec', 1.0)
        self.declare_parameter('max_solver_depth', 100)

        # Get parameter values
        self.tape_thickness = self.get_parameter('tape_thickness_m').value
        self.current_grid_row = self.get_parameter('start_row').value
        self.current_grid_col = self.get_parameter('start_col').value
        self.current_orientation_rad = self.get_parameter('start_orientation_rad').value
        self.max_solver_depth = self.get_parameter('max_solver_depth').value
        move_forward_topic = self.get_parameter('move_forward_topic').value
        distance_status_topic = self.get_parameter('distance_status_topic').value
        turn_command_topic = self.get_parameter('turn_command_topic').value
        turn_status_topic = self.get_parameter('turn_status_topic').value
        control_period = 1.0 / self.get_parameter('control_frequency').value
        self.wait_time_after_step = self.get_parameter('wait_time_after_step_sec').value

        # --- HARDCODE the square dimensions data ---
        grid_data = [
            [ {'length': 0.2, 'width': 0.2}]*7, [ {'length': 0.2, 'width': 0.2}]*7,
            [ {'length': 0.2, 'width': 0.2}]*7, [ {'length': 0.2, 'width': 0.2}]*7,
            [ {'length': 0.2, 'width': 0.2}]*7, [ {'length': 0.2, 'width': 0.2}]*7,
            [ {'length': 0.2, 'width': 0.2}]*7
        ]
        self.square_dims = grid_data
        self.rows = len(self.square_dims)
        self.cols = len(self.square_dims[0]) if self.rows > 0 else 0

        # --- Publishers ---
        self.move_pub = self.create_publisher(Float32, move_forward_topic, 10)
        self.turn_pub = self.create_publisher(String, turn_command_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # --- Subscribers ---
        self.dist_sub = self.create_subscription(
            String, distance_status_topic, self.distance_status_callback, 10)
        self.turn_sub = self.create_subscription(
            String, turn_status_topic, self.turn_status_callback, 10)

        # --- State Variables ---
        self.current_state = self.IDLE
        self.solver_instance: Optional[AStar] = None
        self.relative_move_list: Optional[List[str]] = None
        self.absolute_move_list: Optional[List[RobotDirection]] = None
        self.current_move_index = 0
        self.absolute_move_index = 0
        self.action_in_progress = False
        self.last_state_transition_time = self.get_clock().now()
        # ** Flags for UTURN sequence **
        self.uturn_part1_pending = False # Flag to indicate we need to start the first turn
        self.uturn_part2_pending = False # Flag to indicate we need to start the second turn

        # --- Timer for Control Loop ---
        self.control_timer = self.create_timer(control_period, self.control_loop_callback)

        self.get_logger().info("Sokoban Navigator Node Initialized.")
        self.get_logger().info(f" Start Pos: ({self.current_grid_row},{self.current_grid_col}), Orientation: {math.degrees(self.current_orientation_rad):.1f} deg")
        self.get_logger().info(f" Grid Size: {self.rows}x{self.cols}")
        self.get_logger().info(f" Pub move_cmd: '{move_forward_topic}', turn_cmd: '{turn_command_topic}'")
        self.get_logger().info(f" Sub status_dist: '{distance_status_topic}', status_turn: '{turn_status_topic}'")
        self.get_logger().info("Starting Solver...")

        # --- Initialize and Run Solver ---
        self.current_state = self.SOLVING
        self.run_solver_and_get_moves()

    def get_square_dims(self, r, c):
        if 0 <= r < self.rows and 0 <= c < self.cols:
            if hasattr(self, 'square_dims') and isinstance(self.square_dims, list) and \
               len(self.square_dims) > r and isinstance(self.square_dims[r], list) and \
               len(self.square_dims[r]) > c and isinstance(self.square_dims[r][c], dict):
                return self.square_dims[r][c]
            else: self.get_logger().error(f"Square dimensions data invalid for ({r},{c})."); return None
        else: self.get_logger().error(f"Attempted to access invalid square ({r},{c}) dimensions!"); return None

    def run_solver_and_get_moves(self):
        self.get_logger().info("Initializing A* Solver...")
        try:
            self.solver_instance = AStar(sol_depth_max=self.max_solver_depth)
            self.get_logger().info("Running A* algorithm...")
            self.solver_instance.solve(log=True)

            if self.solver_instance.solved:
                self.get_logger().info("Solver found a solution!")
                self.relative_move_list = self.solver_instance.relative_robot_moves_detailed
                self.absolute_move_list = self.solver_instance.final_robot_moves
                if not self.relative_move_list or not self.absolute_move_list:
                    self.get_logger().error("Solver success but move list(s) empty!")
                    self.current_state = self.FAILED
                else:
                    self.get_logger().info(f"Solution has {len(self.relative_move_list)} relative steps.")
                    log_limit = 15
                    rel_moves_str = str(self.relative_move_list[:log_limit]) + ('...' if len(self.relative_move_list) > log_limit else '')
                    abs_moves_str = str([m.name for m in self.absolute_move_list[:log_limit]]) + ('...' if len(self.absolute_move_list) > log_limit else '')
                    self.get_logger().info(f"Relative Moves: {rel_moves_str}")
                    self.get_logger().info(f"Absolute Moves: {abs_moves_str}")
                    self.current_move_index = 0
                    self.absolute_move_index = 0
                    self.get_logger().info("Waiting briefly before starting execution...")
                    time.sleep(2.0) # Give other nodes time to start
                    self.current_state = self.EXECUTING_MOVE
                    self.get_logger().info("State -> EXECUTING_MOVE")
            else:
                self.get_logger().error("Solver did not find a solution.")
                self.current_state = self.FAILED

        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize or run A* solver: {e}")
            traceback.print_exc()
            self.current_state = self.FAILED

    def distance_status_callback(self, msg: String):
        if msg.data.lower() == "finished":
            self.get_logger().debug("Received 'finished' status for distance.")
            if self.current_state == self.AWAITING_MOVE:
                self.get_logger().info("Move action completed.")
                self.action_in_progress = False
                if self.absolute_move_list and self.absolute_move_index < len(self.absolute_move_list):
                    last_move_direction = self.absolute_move_list[self.absolute_move_index]
                    if last_move_direction == RobotDirection.UP: self.current_grid_row -= 1
                    elif last_move_direction == RobotDirection.DOWN: self.current_grid_row += 1
                    elif last_move_direction == RobotDirection.LEFT: self.current_grid_col -= 1
                    elif last_move_direction == RobotDirection.RIGHT: self.current_grid_col += 1
                    self.absolute_move_index += 1
                    self.get_logger().info(f"Updated position after move: ({self.current_grid_row}, {self.current_grid_col}). Abs move index: {self.absolute_move_index}")
                else: self.get_logger().error("Could not update position - absolute move list issue.")
                self.current_state = self.WAITING_AFTER_STEP
                self.last_state_transition_time = self.get_clock().now()
                self.get_logger().info(f"State -> WAITING_AFTER_STEP (Wait: {self.wait_time_after_step}s)")
            else: self.get_logger().warn(f"Received 'finished' distance status while not in AWAITING_MOVE state (State: {self.current_state}). Ignoring.")
        elif "error" in msg.data.lower():
             self.get_logger().error(f"Received error status from distance controller: {msg.data}. State -> FAILED.")
             self.current_state = self.FAILED
             self.action_in_progress = False

    def turn_status_callback(self, msg: String):
        if msg.data.lower() == "finished":
            self.get_logger().debug("Received 'finished' status for turn.")
            if self.current_state == self.AWAITING_TURN:
                self.get_logger().info("Turn action completed.")
                self.action_in_progress = False
                # Check if this was part of a UTURN
                if self.uturn_part1_pending:
                     self.get_logger().info("UTURN Part 1 finished. Flagging Part 2 needed.")
                     self.uturn_part1_pending = False # Clear part 1 flag
                     self.uturn_part2_pending = True  # Set part 2 flag
                     # Go back to execute loop immediately to trigger part 2
                     self.current_state = self.EXECUTING_MOVE
                     self.get_logger().info("State -> EXECUTING_MOVE (for UTURN part 2)")
                elif self.uturn_part2_pending:
                     self.get_logger().info("UTURN Part 2 finished.")
                     self.uturn_part2_pending = False # Clear part 2 flag
                     # Now the full UTURN relative move is done, go to wait state
                     self.current_state = self.WAITING_AFTER_STEP
                     self.last_state_transition_time = self.get_clock().now()
                     self.get_logger().info(f"State -> WAITING_AFTER_STEP (Wait: {self.wait_time_after_step}s)")
                else:
                    # Normal turn finished, transition to wait state
                    self.current_state = self.WAITING_AFTER_STEP
                    self.last_state_transition_time = self.get_clock().now()
                    self.get_logger().info(f"State -> WAITING_AFTER_STEP (Wait: {self.wait_time_after_step}s)")
            else: self.get_logger().warn(f"Received 'finished' turn status while not in AWAITING_TURN state (State: {self.current_state}). Ignoring.")
        elif "error" in msg.data.lower():
             self.get_logger().error(f"Received error status from turn controller: {msg.data}. State -> FAILED.")
             self.current_state = self.FAILED
             self.action_in_progress = False
             self.uturn_part1_pending = False # Clear flags on error
             self.uturn_part2_pending = False

    def send_turn_command(self, direction: str):
        """ Sends 'left' or 'right' turn command and updates internal orientation. """
        if self.action_in_progress:
            self.get_logger().warn("Action already in progress, cannot send turn command.")
            return False
        if direction.lower() not in ["left", "right"]:
            self.get_logger().error(f"Invalid turn direction for controller: {direction}")
            return False

        cmd = String(); cmd.data = direction.lower()
        angle_change = math.pi / 2.0 if direction.lower() == "left" else -math.pi / 2.0
        self.current_orientation_rad = normalize_angle(self.current_orientation_rad + angle_change)
        self.get_logger().info(f"Updating internal orientation for '{direction}' turn to: {math.degrees(self.current_orientation_rad):.1f} deg")

        self.turn_pub.publish(cmd)
        self.action_in_progress = True
        self.current_state = self.AWAITING_TURN
        self.get_logger().info(f"Published turn command: '{cmd.data}'. State -> AWAITING_TURN")
        return True

    def send_move_command(self, distance: float):
        """ Sends move forward command. """
        if self.action_in_progress:
            self.get_logger().warn("Action already in progress, cannot send move command.")
            return False
        if distance <= 0.001:
            self.get_logger().warn(f"Attempting to move near-zero distance ({distance:.4f}). Skipping move, advancing state.")
            self.current_state = self.WAITING_AFTER_STEP
            self.last_state_transition_time = self.get_clock().now()
            self.get_logger().info(f"State -> WAITING_AFTER_STEP (Wait: {self.wait_time_after_step}s)")
            if self.absolute_move_list and self.absolute_move_index < len(self.absolute_move_list):
                 self.absolute_move_index += 1
                 self.get_logger().info(f"Advanced absolute index after skipping move: {self.absolute_move_index}")
            return True

        cmd = Float32(); cmd.data = distance
        self.move_pub.publish(cmd)
        self.action_in_progress = True
        self.current_state = self.AWAITING_MOVE
        self.get_logger().info(f"Published move command: {cmd.data:.3f}m. State -> AWAITING_MOVE")
        return True

    def calculate_forward_distance(self, move_direction: RobotDirection) -> float:
        """ Calculates the distance needed for a FORWARD move based on the absolute direction. """
        target_r, target_c = self.current_grid_row, self.current_grid_col
        delta_r, delta_c = 0, 0
        if move_direction == RobotDirection.UP: delta_r = -1
        elif move_direction == RobotDirection.DOWN: delta_r = 1
        elif move_direction == RobotDirection.LEFT: delta_c = -1
        elif move_direction == RobotDirection.RIGHT: delta_c = 1
        target_r += delta_r; target_c += delta_c

        current_dims = self.get_square_dims(self.current_grid_row, self.current_grid_col)
        target_dims = self.get_square_dims(target_r, target_c)
        if not current_dims or not target_dims:
            self.get_logger().error(f"Could not get square dimensions. Current:({self.current_grid_row},{self.current_grid_col}), Target:({target_r},{target_c})")
            return 0.0

        distance = 0.0
        if delta_r != 0: distance = (current_dims['length'] / 2.0) + self.tape_thickness + (target_dims['length'] / 2.0)
        elif delta_c != 0: distance = (current_dims['width'] / 2.0) + self.tape_thickness + (target_dims['width'] / 2.0)
        self.get_logger().info(f"Calculated move distance for {move_direction.name}: {distance:.3f}m")
        return distance

    def control_loop_callback(self):
        """ Main state machine logic for executing the solved plan. """
        if self.current_state in [self.IDLE, self.SOLVING, self.AWAITING_TURN, self.AWAITING_MOVE, self.FINISHED, self.FAILED]:
            return

        # --- State: WAITING_AFTER_STEP ---
        if self.current_state == self.WAITING_AFTER_STEP:
            elapsed_time = (self.get_clock().now() - self.last_state_transition_time).nanoseconds / 1e9
            if elapsed_time >= self.wait_time_after_step:
                self.get_logger().info("Wait time finished.")
                # Increment relative move index AFTER waiting and BEFORE executing next move
                self.current_move_index += 1
                self.current_state = self.EXECUTING_MOVE
                self.get_logger().info("State -> EXECUTING_MOVE")
            else: return # Still waiting

        # --- State: EXECUTING_MOVE ---
        elif self.current_state == self.EXECUTING_MOVE:
            if self.action_in_progress:
                self.get_logger().warn("In EXECUTING_MOVE state but action_in_progress is True? Waiting.", throttle_duration_sec=5.0)
                return

            # *** Check if second part of UTURN is needed FIRST ***
            if self.uturn_part2_pending:
                self.get_logger().info("Executing UTURN Part 2 (sending second turn command).")
                # Send the second turn command (e.g., left again)
                if not self.send_turn_command("left"): # State becomes AWAITING_TURN
                     self.get_logger().error("Failed to send second turn command for UTURN. State -> FAILED")
                     self.current_state = self.FAILED
                     self.uturn_part2_pending = False # Clear flag on error
                # Note: uturn_part2_pending flag will be cleared in the callback when this turn finishes
                return # End processing for this cycle, wait for turn to finish

            # *** If not doing UTURN part 2, proceed with normal move list ***
            if not self.relative_move_list: self.get_logger().error("No move list!"); self.current_state = self.FAILED; return
            if self.current_move_index >= len(self.relative_move_list):
                self.get_logger().info("All moves executed successfully!")
                self.current_state = self.FINISHED; return

            next_relative_move = self.relative_move_list[self.current_move_index]
            self.get_logger().info(f"Executing Step {self.current_move_index + 1}/{len(self.relative_move_list)}: Relative Move = '{next_relative_move}'")

            # --- Handle Moves ---
            if next_relative_move == TURN_LEFT:
                self.send_turn_command("left")
            elif next_relative_move == TURN_RIGHT:
                self.send_turn_command("right")
            elif next_relative_move == UTURN:
                self.get_logger().info("UTURN command encountered. Initiating Part 1 (sending first turn).")
                self.uturn_part1_pending = True # Set flag for callback
                if not self.send_turn_command("left"): # Send first turn
                     self.get_logger().error("Failed to send first turn command for UTURN. State -> FAILED")
                     self.current_state = self.FAILED
                     self.uturn_part1_pending = False # Clear flag on error
            elif next_relative_move == FORWARD:
                if self.absolute_move_list and self.absolute_move_index < len(self.absolute_move_list):
                    absolute_direction = self.absolute_move_list[self.absolute_move_index]
                    required_orientation = self.ROBOTDIR_TO_ORIENTATION.get(absolute_direction)
                    if required_orientation is None: self.get_logger().error(f"Cannot map abs dir {absolute_direction.name}. FAILED"); self.current_state = self.FAILED; return
                    angle_diff = normalize_angle(required_orientation - self.current_orientation_rad)
                    if abs(angle_diff) > math.radians(10.0): self.get_logger().error(f"Orientation mismatch! Current:{math.degrees(self.current_orientation_rad):.1f}, Required:{math.degrees(required_orientation):.1f}. FAILED"); self.current_state = self.FAILED; return
                    else:
                        distance = self.calculate_forward_distance(absolute_direction)
                        if distance > 0: self.send_move_command(distance)
                        else: self.get_logger().error(f"Zero distance for FORWARD ({absolute_direction.name}). FAILED."); self.current_state = self.FAILED; return
                else: self.get_logger().error("Absolute move list issue for FORWARD. FAILED."); self.current_state = self.FAILED; return
            else: self.get_logger().error(f"Unknown relative move: '{next_relative_move}'. FAILED."); self.current_state = self.FAILED; return

            # ** Move index increment moved to WAITING_AFTER_STEP state transition **

    def shutdown_hook(self):
        self.get_logger().info("Sokoban Navigator shutting down...")
        if hasattr(self, 'control_timer') and self.control_timer: self.control_timer.cancel()
        stop_msg = Twist()
        try:
             if hasattr(self, 'cmd_vel_pub') and self.cmd_vel_pub:
                  self.cmd_vel_pub.publish(stop_msg)
                  self.get_logger().info("Sent final stop command to /cmd_vel.")
                  time.sleep(0.1)
             else: self.get_logger().warn("Could not send final stop command (cmd_vel_pub not ready?).")
        except Exception as e: self.get_logger().error(f"Exception during shutdown stop command: {e}")

def main(args=None):
    rclpy.init(args=args)
    sokoban_navigator_node = None
    try:
        sokoban_navigator_node = SokobanNavigatorNode()
        rclpy.spin(sokoban_navigator_node)
    except KeyboardInterrupt: print('Keyboard interrupt, shutting down.')
    except SystemExit: print('Shutdown initiated by node.')
    except Exception as e:
        if sokoban_navigator_node: sokoban_navigator_node.get_logger().fatal(f"Unhandled exception: {e}"); traceback.print_exc()
        else: print(f"Unhandled exception before node init: {e}"); traceback.print_exc()
    finally:
        if sokoban_navigator_node:
            sokoban_navigator_node.shutdown_hook()
            sokoban_navigator_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        print("Sokoban Navigator Shutdown complete.")

if __name__ == '__main__':
    main()
