# -*- coding: utf-8 -*-
# Referenced file: sk_utilities.py
# Referenced file: sk_solver_util.py
from .sk_utilities import Tile, RobotDirection, Push # Add dot
from .sk_solver_util import SearchAlgorithm, AStarHeuristic, AStarNode, OpenListAStar # Add dot
from .sk_game import SokobanGame, SokobanLogic # Add dot
import time
# Referenced file: sk_game.py
import copy
import logging
import pickle
import numpy as np
from typing import List, Tuple, Optional


# Add Matplotlib imports
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# Keep interactive mode on for event handling during pauses
plt.ion()

# --- Constants for Relative Moves ---
FORWARD = "FORWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
UTURN = "UTURN"
# --- End Constants ---

# --- Global flag for step control ---
_proceed_to_next_step = False
_user_closed_window = False

# --- Key press event handler ---
def _on_key_press(event):
    """Handles key press events for stepping through the visualization."""
    global _proceed_to_next_step
    # Use spacebar or 'n' to advance
    if event.key == ' ' or event.key == 'n':
        _proceed_to_next_step = True
    # Optional: Add a key to quit early, e.g., 'q'
    elif event.key == 'q':
        global _user_closed_window
        _user_closed_window = True # Treat 'q' as closing the window
        _proceed_to_next_step = True # Allow loop to exit


# --- COLOR MAP and get_tile_color function (Keep as before) ---
TILE_COLORS = {
    Tile.WALL.value: (0.3, 0.3, 0.3), Tile.FLOOR.value: (0.9, 0.9, 0.9), Tile.GOAL.value: (1.0, 0.8, 0.8),
    (Tile.FLOOR.value | Tile.ROBOT.value): (0.0, 0.5, 1.0),
    (Tile.FLOOR.value | Tile.BOX.value): (0.8, 0.4, 0.0),
    (Tile.GOAL.value | Tile.BOX.value): (0.5, 1.0, 0.5),
    (Tile.GOAL.value | Tile.ROBOT.value): (0.0, 0.8, 0.8),
    16: (0.3, 0.3, 0.3), 2: (0.9, 0.9, 0.9), 6: (1.0, 0.8, 0.8),
    10: (0.0, 0.5, 1.0), 3: (0.8, 0.4, 0.0), 7: (0.5, 1.0, 0.5),
}
DEFAULT_COLOR = (1.0, 0.0, 1.0)

def get_tile_color(tile_val):
    """Gets the background color for a tile (Wall, Floor, or Goal)."""
    tile_val = int(tile_val)
    is_wall = bool(tile_val & Tile.WALL.value)
    is_goal = bool(tile_val & Tile.GOAL.value)
    if is_wall: return TILE_COLORS[Tile.WALL.value]
    if is_goal: return TILE_COLORS[Tile.GOAL.value]
    return TILE_COLORS[Tile.FLOOR.value]

# --- Map RobotDirection to Matplotlib Markers (Keep as before) ---
ROBOT_MARKERS = {
    RobotDirection.UP: '^', RobotDirection.DOWN: 'v',
    RobotDirection.LEFT: '<', RobotDirection.RIGHT: '>'
}
ROBOT_MARKER_COLOR = (0.0, 0.0, 1.0)
BOX_COLOR = TILE_COLORS[(Tile.FLOOR.value | Tile.BOX.value)]
BOX_ON_GOAL_COLOR = TILE_COLORS[(Tile.GOAL.value | Tile.BOX.value)]

# --- Matplotlib Figure Setup (Keep as before) ---
fig_vis, ax_vis = plt.subplots(figsize=(8, 8))
imshow_obj = None
robot_marker_obj = None
box_patches = []

# --- END DEFINITIONS ---


class AStar(SearchAlgorithm):
    # __init__ remains the same as the previous version
    def __init__(self, sol_depth_max:int):
        # Referenced file: sk_solver_util.py
        super().__init__(file_map_name="<hardcoded>", sol_depth_max=sol_depth_max)
        self.name = "A* Best First Search"
        try:
            # Referenced file: sk_game.py
            self.game = SokobanGame()
        except Exception as e:
            # Updated error logging as file name is no longer the direct cause
            logging.error(f"A* Init Error: Failed to load game (hardcoded map): {e}")
            raise
        # except FileNotFoundError:
        #     logging.error(f"A* Init Error: Map file '{}' not found.")
        #     raise
        # except Exception as e:
        #     logging.error(f"A* Init Error: Failed to load game from '{}': {e}")
        #     raise

        # Referenced file: sk_solver_util.py
        self.logic = AStarHeuristic(
            # Referenced file: sk_game.py
            grid_shape=(self.game.grid_rows, self.game.grid_cols),
            wall_indices=self.game.wall_index,
            step_fore=self.game.step_fore,
            goal_indices=self.game.goals_index
        )
        # Referenced file: sk_solver_util.py
        self.open_list = OpenListAStar()
        self.closed_list = set()
        self.n_pushes_evaled = 0
        self.n_pushes_skipped = 0
        # Stores the final sequence of absolute moves (UP, DOWN, etc.)
        self.final_robot_moves: Optional[List[RobotDirection]] = None
        # Stores the detailed sequence of relative moves (TURN_*, FORWARD)
        self.relative_robot_moves_detailed: Optional[List[str]] = None

    # _get_canonical_robot_pos remains the same
    def _get_canonical_robot_pos(self, grid: np.ndarray, robot_pos: int, boxes_pos: np.ndarray) -> int:
        """ Finds the minimal reachable tile index for the robot for state hashing. """
        # Referenced file: sk_game.py (SokobanLogic method)
        _reachable, _potential = self.logic.calc_reachable_tiles_and_potential_pushes(grid, robot_pos, boxes_pos)
        # Referenced file: sk_game.py (SokobanLogic attribute)
        if self.logic.min_reachable_tile == -1:
             return robot_pos # Fallback if calculation fails
        # Referenced file: sk_game.py (SokobanLogic attribute)
        return self.logic.min_reachable_tile

    # _extract_absolute_moves_from_path remains the same
    def _extract_absolute_moves_from_path(self, path: List[Tuple[List[RobotDirection], Push]]) -> List[RobotDirection]:
        """ Extracts the complete sequence of absolute robot moves from the solution path. """
        absolute_moves: List[RobotDirection] = []
        for moves_before_push, push_action in path:
            if moves_before_push:
                absolute_moves.extend(moves_before_push)
            # Add the move corresponding to the push direction
            try:
                # Referenced file: sk_utilities.py
                push_direction_enum = RobotDirection(push_action.direction)
                absolute_moves.append(push_direction_enum)
            except (ValueError, AttributeError) as e:
                 logging.error(f"Error extracting push direction: {e}. Push Action: {push_action}")
                 continue
        return absolute_moves

    # _convert_absolute_to_relative remains the same
    def _convert_absolute_to_relative(self, absolute_moves: List[RobotDirection], initial_direction: RobotDirection = RobotDirection.UP) -> List[str]:
        """ Converts a list of absolute moves to a detailed list of relative moves (TURN_*, FORWARD). """
        if not absolute_moves:
            return []

        relative_moves_detailed: List[str] = []
        current_direction = initial_direction

        # Logic to determine turns based on current and target direction
        turn_logic = {
            (RobotDirection.UP, RobotDirection.UP): (None, RobotDirection.UP),
            (RobotDirection.UP, RobotDirection.DOWN): (UTURN, RobotDirection.DOWN),
            (RobotDirection.UP, RobotDirection.LEFT): (TURN_LEFT, RobotDirection.LEFT),
            (RobotDirection.UP, RobotDirection.RIGHT): (TURN_RIGHT, RobotDirection.RIGHT),
            (RobotDirection.DOWN, RobotDirection.UP): (UTURN, RobotDirection.UP),
            (RobotDirection.DOWN, RobotDirection.DOWN): (None, RobotDirection.DOWN),
            (RobotDirection.DOWN, RobotDirection.LEFT): (TURN_RIGHT, RobotDirection.LEFT),
            (RobotDirection.DOWN, RobotDirection.RIGHT): (TURN_LEFT, RobotDirection.RIGHT),
            (RobotDirection.LEFT, RobotDirection.UP): (TURN_RIGHT, RobotDirection.UP),
            (RobotDirection.LEFT, RobotDirection.DOWN): (TURN_LEFT, RobotDirection.DOWN),
            (RobotDirection.LEFT, RobotDirection.LEFT): (None, RobotDirection.LEFT),
            (RobotDirection.LEFT, RobotDirection.RIGHT): (UTURN, RobotDirection.RIGHT),
            (RobotDirection.RIGHT, RobotDirection.UP): (TURN_LEFT, RobotDirection.UP),
            (RobotDirection.RIGHT, RobotDirection.DOWN): (TURN_RIGHT, RobotDirection.DOWN),
            (RobotDirection.RIGHT, RobotDirection.LEFT): (UTURN, RobotDirection.LEFT),
            (RobotDirection.RIGHT, RobotDirection.RIGHT): (None, RobotDirection.RIGHT),
        }

        for target_absolute_move in absolute_moves:
            if not isinstance(target_absolute_move, RobotDirection):
                logging.warning(f"Skipping non-RobotDirection item in absolute moves: {target_absolute_move}")
                continue
            turn_needed, next_direction_after_turn = turn_logic.get(
                (current_direction, target_absolute_move), (None, current_direction)
            )
            if turn_needed:
                relative_moves_detailed.append(turn_needed)
                current_direction = next_direction_after_turn
            relative_moves_detailed.append(FORWARD)
            current_direction = target_absolute_move
        return relative_moves_detailed

    # algorithm remains the same
    def algorithm(self):
        """ A* search algorithm implementation. """
        # Initial state
        # Referenced file: sk_game.py
        initial_robot_pos = self.game.robot_index
        initial_boxes_pos = self.game.boxes_index.copy()
        # Referenced file: sk_solver_util.py
        initial_heuristic = self.logic.calculate_simple_lower_bound(initial_boxes_pos)

        if initial_heuristic == np.iinfo(np.int32).max:
            logging.warning("Initial state is already unsolvable (dead end based on heuristic).")
            return # Cannot solve

        # Referenced file: sk_game.py
        initial_canonical_robot_pos = self._get_canonical_robot_pos(self.game.grid, initial_robot_pos, initial_boxes_pos)
        # Referenced file: sk_game.py
        initial_ztup = (initial_canonical_robot_pos, self.game.state_zhash)

        # Referenced file: sk_solver_util.py
        start_node = AStarNode(
            ztup=initial_ztup, pred=None, robot_moves_before_push=None, push=None,
            push_depth=0, lower_bound=initial_heuristic,
            # Referenced file: sk_game.py
            game_state=(self.game.robot_index, initial_boxes_pos)
        )
        # Referenced file: sk_solver_util.py
        self.open_list.add_node(start_node)
        logging.info(f"Starting A* search. Initial h={initial_heuristic}")

        # Main loop
        # Referenced file: sk_solver_util.py
        while not self.open_list.is_empty() and not self.solved:
            # Referenced file: sk_solver_util.py
            node = self.open_list.pop_node()
            if node is None:
                logging.info("A* Search: Open list became empty.")
                break

            current_robot_index, current_boxes_index = node.game_state
            try: # Referenced file: sk_game.py
                self.game.set_game_state(current_robot_index, current_boxes_index)
            except IndexError:
                 logging.error(f"Error setting game state! Robot:{current_robot_index}, Boxes:{current_boxes_index}. Node depth: {node.push_depth}. Skipping node.")
                 continue

            # Referenced file: sk_game.py
            canonical_robot_pos = self._get_canonical_robot_pos(self.game.grid, current_robot_index, current_boxes_index)
            # Referenced file: sk_game.py
            current_ztup = (canonical_robot_pos, self.game.state_zhash)

            if current_ztup in self.closed_list:
                self.n_pushes_skipped += 1; continue

            self.closed_list.add(current_ztup)
            self.n_pushes_evaled += 1
            if self.n_pushes_evaled % 1000 == 0: # Referenced file: sk_solver_util.py
                 logging.info(f" A*: Expanded {self.n_pushes_evaled} nodes. Open list size: {len(self.open_list.heap)}. Current depth: {node.push_depth}. f={node.f}, h={node.lower_bound}")

            # Goal Check # Referenced file: sk_game.py
            if self.game.is_solved():
                logging.info(f"A* Search: Solution Found! Push Depth: {node.push_depth}")
                self.solved = True; self.count_depth = node.push_depth
                self.solution = node.path
                self.final_robot_moves = self._extract_absolute_moves_from_path(node.path)
                self.relative_robot_moves_detailed = self._convert_absolute_to_relative(self.final_robot_moves)
                break

            # Depth Limit Check
            if node.push_depth >= self.sol_depth_max: continue

            # Expand Node # Referenced file: sk_game.py
            pushes_info = self.logic.calc_available_pushes(
                self.game.grid, current_robot_index, current_boxes_index
            )

            # Process potential pushes # Referenced file: sk_utilities.py
            for push_action, robot_pos_needed in pushes_info:
                 # Referenced file: sk_game.py
                robot_path_segment = self.logic.find_robot_path(
                    self.game.grid, current_robot_index, robot_pos_needed, current_boxes_index
                )
                if robot_path_segment is not None:
                    self.game.move_box(push_action) # Referenced file: sk_game.py
                    next_robot_index = self.game.robot_index # Referenced file: sk_game.py
                    next_boxes_index = self.game.boxes_index.copy() # Referenced file: sk_game.py
                    # Referenced file: sk_solver_util.py
                    next_heuristic = self.logic.calculate_simple_lower_bound(next_boxes_index)

                    if next_heuristic >= np.iinfo(np.int32).max:
                         self.game.unmove_box(push_action); continue # Referenced file: sk_game.py

                    # Referenced file: sk_game.py
                    next_canonical_robot_pos = self._get_canonical_robot_pos(self.game.grid, next_robot_index, next_boxes_index)
                    # Referenced file: sk_game.py
                    next_ztup = (next_canonical_robot_pos, self.game.state_zhash)
                    # Referenced file: sk_solver_util.py
                    neighbour = AStarNode(
                        ztup=next_ztup, pred=node, robot_moves_before_push=robot_path_segment,
                        push=push_action, push_depth=node.push_depth + 1, # Referenced file: sk_utilities.py
                        lower_bound=next_heuristic, game_state=(next_robot_index, next_boxes_index)
                    )
                    self.open_list.add_node(neighbour) # Referenced file: sk_solver_util.py
                    self.game.unmove_box(push_action) # Referenced file: sk_game.py
        # --- End of A* search loop ---


    def wait_for_step(self, prompt="Press SPACE or 'n' to continue, 'q' to quit..."):
        """ Pauses execution and waits for a key press in the plot window. """
        global _proceed_to_next_step, _user_closed_window
        logging.info(prompt) # Log prompt to console as well
        _proceed_to_next_step = False # Reset flag before waiting

        original_title = ax_vis.get_title()
        ax_vis.set_title(f"{original_title}\n{prompt}")
        fig_vis.canvas.draw_idle()

        while not _proceed_to_next_step:
            # Check if the window was closed externally
            if not plt.fignum_exists(fig_vis.number):
                logging.warning("Visualization window closed by user.")
                _user_closed_window = True
                break
            # Process events and pause briefly
            plt.pause(0.1)

        # Restore original title part after key press (or window close)
        ax_vis.set_title(original_title)
        fig_vis.canvas.draw_idle()

        # Return False if user quit or closed window, True otherwise
        return not _user_closed_window


    def visualize_step(self, game: SokobanGame, step_num: int, move_name: str, robot_orientation: RobotDirection):
        """ Visualizes the current grid state. Does NOT pause. """
        global imshow_obj, robot_marker_obj, box_patches, fig_vis, ax_vis, _user_closed_window

        # Check if the figure window still exists before drawing
        if _user_closed_window or not plt.fignum_exists(fig_vis.number):
            if not _user_closed_window: # Only log if not already known
                logging.warning("Visualization window closed by user.")
                _user_closed_window = True
            return False # Indicate failure/closure

        grid_shape = (game.grid_rows, game.grid_cols)

        # --- Draw Background ---
        vis_grid_background = np.zeros((*grid_shape, 3), dtype=float)
        for r in range(grid_shape[0]):
            for c in range(grid_shape[1]):
                base_idx = r * grid_shape[1] + c
                base_tile_val = game.grid_base_flat[base_idx]
                vis_grid_background[r, c] = get_tile_color(base_tile_val)

        if imshow_obj is None:
            ax_vis.clear()
            imshow_obj = ax_vis.imshow(vis_grid_background, interpolation='nearest', aspect='equal', zorder=0, origin='upper')
            ax_vis.set_xticks(np.arange(-.5, grid_shape[1], 1), minor=True)
            ax_vis.set_yticks(np.arange(-.5, grid_shape[0], 1), minor=True)
            ax_vis.grid(which="minor", color="k", linestyle='-', linewidth=0.5)
            ax_vis.tick_params(which="minor", size=0)
            ax_vis.set_xticks(np.arange(0, grid_shape[1], 1))
            ax_vis.set_yticks(np.arange(0, grid_shape[0], 1))
            ax_vis.xaxis.tick_top()
            ax_vis.tick_params(axis='both', which='major', labelsize=8)
        else:
            imshow_obj.set_data(vis_grid_background)

        # --- Draw Boxes ---
        for patch in box_patches: patch.remove()
        box_patches.clear()
        # Referenced file: sk_game.py
        for box_idx in game.boxes_index:
            r, c = np.unravel_index(box_idx, grid_shape)
            # Referenced file: sk_game.py
            box_color = BOX_ON_GOAL_COLOR if box_idx in game.goals_index else BOX_COLOR
            rect = plt.Rectangle((c - 0.5, r - 0.5), 1, 1, facecolor=box_color, edgecolor='black', linewidth=1.5, zorder=1)
            ax_vis.add_patch(rect)
            box_patches.append(rect)

        # --- Draw Robot Marker ---
        # Referenced file: sk_game.py
        robot_r, robot_c = np.unravel_index(game.robot_index, grid_shape)
        marker = ROBOT_MARKERS.get(robot_orientation, '?')

        if robot_marker_obj is None:
             line_list = ax_vis.plot(robot_c, robot_r, marker=marker, color=ROBOT_MARKER_COLOR,
                                markersize=12, linestyle='None', zorder=2, markeredgewidth=1.5, markeredgecolor='black')
             if line_list: robot_marker_obj = line_list[0]
             else: logging.error("Failed to create robot marker plot object."); return False
        else:
            robot_marker_obj.set_data([robot_c], [robot_r])
            robot_marker_obj.set_marker(marker)

        # --- Update Title ---
        title_str = f"Relative Step {step_num}: {move_name}"
        # Referenced file: sk_game.py
        if game.is_solved(): title_str += " - SOLVED!"
        ax_vis.set_title(title_str) # Set base title (wait prompt added in wait_for_step)

        # Adjust plot limits
        ax_vis.set_xlim(-0.5, grid_shape[1] - 0.5)
        ax_vis.set_ylim(grid_shape[0] - 0.5, -0.5)

        # --- Draw Updates ---
        fig_vis.canvas.draw_idle()
        return True # Indicate success

    def solve(self, log=False) -> None:
        """ Solves the Sokoban puzzle using A*. Does NOT visualize. """
        logging.info(f"Solver: {self.name}")
        self.print_solver_info(log) # Keep this if useful
        # ... (keep initial grid logging if useful) ...
        # ... (keep heuristic distance logging if useful) ...

        # --- Run A* Search ---
        start_time = time.time()
        self.algorithm() # This runs the core search
        end_time = time.time()
        execution_time = end_time - start_time
        # --- Search Finished ---

        logging.info(f"\n--- Search Finished ---")
        logging.info(f"Execution time: {execution_time:.4f} seconds")
        logging.info(f"Solved: {self.solved}")

        if self.solved:
            logging.info(f"Solution push depth: {self.count_depth}")
            logging.info(f"Nodes expanded: {self.n_pushes_evaled}")
            logging.info(f"Nodes skipped: {self.n_pushes_skipped}")

            # Log moves
            if self.final_robot_moves:
                absolute_move_names = [move.name for move in self.final_robot_moves if isinstance(move, RobotDirection)]
                logging.info(f"Total absolute robot moves in solution: {len(absolute_move_names)}")
                logging.info(f"Absolute move list: {absolute_move_names}")
            if self.relative_robot_moves_detailed:
                logging.info(f"Total relative steps in solution: {len(self.relative_robot_moves_detailed)}")
                logging.info(f"Relative move list: {self.relative_robot_moves_detailed}")

            # Save Solution Files
            try:
                # Ensure solution paths are not None before saving
                if self.solution is not None:
                    with open("sokoban_solution_astar_detailed_path.pkl", "wb") as f: pickle.dump(self.solution, f)
                if self.final_robot_moves is not None:
                    with open("sokoban_solution_astar_moves_absolute.pkl", "wb") as f: pickle.dump(self.final_robot_moves, f)
                if self.relative_robot_moves_detailed is not None:
                    with open("sokoban_solution_astar_moves_relative_detailed.pkl", "wb") as f: pickle.dump(self.relative_robot_moves_detailed, f)
                logging.info("Solution details saved to pickle files.")
            except Exception as e:
                logging.error(f"Error saving solution to pickle files: {e}")
        else:
            logging.info(f"No solution found within depth {self.sol_depth_max}")
            logging.info(f"Nodes expanded: {self.n_pushes_evaled}")
            logging.info(f"Nodes skipped: {self.n_pushes_skipped}")

    # def solve_with_plt(self, log=False) -> None:
    # # def solve(self, log=False) -> None:
    #     """ Solves the Sokoban puzzle using A* and visualizes the relative steps with manual control. """
    #     global imshow_obj, robot_marker_obj, box_patches, fig_vis, ax_vis
    #     global _proceed_to_next_step, _user_closed_window # Use global flags

    #     # Reset plot objects and flags at the start of solve
    #     imshow_obj = None
    #     robot_marker_obj = None
    #     box_patches = []
    #     ax_vis.clear()
    #     _proceed_to_next_step = False
    #     _user_closed_window = False

    #     logging.info(f"Solver: {self.name}")
    #     # Referenced file: sk_solver_util.py
    #     self.print_solver_info(log)
    #     logging.info("--- Initial Grid (Console) ---")
    #     # Referenced file: sk_game.py
    #     self.game.print_grid(log=True)
    #     logging.info("--- Heuristic Distances ---")
    #     try: # Referenced file: sk_solver_util.py
    #         self.logic.print_distance_grid()
    #     except Exception as e: logging.error(f"Error printing distance grid: {e}")

    #     # --- Run A* Search ---
    #     start_time = time.time()
    #     self.algorithm()
    #     end_time = time.time()
    #     execution_time = end_time - start_time
    #     # --- Search Finished ---

    #     logging.info(f"\n--- Search Finished ---")
    #     logging.info(f"Execution time: {execution_time:.4f} seconds")
    #     logging.info(f"Solved: {self.solved}")

    #     if self.solved:
    #         logging.info(f"Solution push depth: {self.count_depth}")
    #         logging.info(f"Nodes expanded: {self.n_pushes_evaled}")
    #         logging.info(f"Nodes skipped: {self.n_pushes_skipped}")

    #         # Log moves (as before)
    #         # Referenced file: sk_utilities.py
    #         absolute_move_names = [move.name for move in self.final_robot_moves if isinstance(move, RobotDirection)]
    #         logging.info(f"Total absolute robot moves in solution: {len(absolute_move_names)}")
    #         logging.info(f"Absolute move list: {absolute_move_names}")
    #         logging.info(f"Total relative steps in solution: {len(self.relative_robot_moves_detailed)}")
    #         logging.info(f"Relative move list: {self.relative_robot_moves_detailed}")

    #         # --- Save Solution Files (as before) ---
    #         try: # Referenced file: sk_solver_util.py
    #             with open("sokoban_solution_astar_detailed_path.pkl", "wb") as f: pickle.dump(self.solution, f)
    #             # Referenced file: sk_solver_util.py
    #             with open("sokoban_solution_astar_moves_absolute.pkl", "wb") as f: pickle.dump(self.final_robot_moves, f)
    #             with open("sokoban_solution_astar_moves_relative_detailed.pkl", "wb") as f: pickle.dump(self.relative_robot_moves_detailed, f)
    #             logging.info("Solution details saved to pickle files.")
    #         except Exception as e: logging.error(f"Error saving solution to pickle files: {e}")


    #         # --- Simulation Loop with Manual Step Control ---
    #         logging.info("\n--- Simulating Solution with Manual Step Visualization ---")
    #         print("\n--- Starting Visualization ---")
    #         print(">>> Focus the plot window and press SPACE or 'n' to advance steps, 'q' to quit. <<<")

    #         # Connect the key press handler
    #         cid = fig_vis.canvas.mpl_connect('key_press_event', _on_key_press)

    #         # Referenced file: sk_game.py
    #         self.game.reset()
    #         relative_step_count = 0
    #         simulation_ok = True
    #         # Referenced file: sk_utilities.py
    #         current_orientation = RobotDirection.UP # Assume starting UP
    #         absolute_move_idx = 0

    #         # Visualize initial state
    #         try:
    #             if not self.visualize_step(self.game, relative_step_count, "Initial State", current_orientation): # Referenced file: sk_game.py
    #                 simulation_ok = False # Window was closed immediately

    #             # Wait for first key press before starting the loop
    #             if simulation_ok:
    #                 if not self.wait_for_step("Press SPACE or 'n' to start simulation..."):
    #                     simulation_ok = False # User quit or closed window

    #             # --- Main Simulation Loop ---
    #             if simulation_ok and self.relative_robot_moves_detailed:
    #                 for relative_move in self.relative_robot_moves_detailed:
    #                     if _user_closed_window: # Check flag before processing step
    #                         simulation_ok = False
    #                         break

    #                     relative_step_count += 1
    #                     vis_title = ""

    #                     # --- Perform Action based on Relative Move ---
    #                     if relative_move == FORWARD:
    #                         if absolute_move_idx < len(self.final_robot_moves):
    #                             # Referenced file: sk_utilities.py
    #                             abs_move = self.final_robot_moves[absolute_move_idx]
    #                             if not isinstance(abs_move, RobotDirection):
    #                                  logging.error(f"Sim Error: Expected RobotDirection, got {type(abs_move)}"); simulation_ok = False; break

    #                             # Referenced file: sk_game.py
    #                             next_pos = self.game.robot_index + self.game.step_fore[abs_move.value]
    #                             is_box_in_front = next_pos in self.game.boxes_index

    #                             move_successful = False
    #                             if is_box_in_front:
    #                                 try: # Implicit Push
    #                                     box_num = np.where(self.game.boxes_index == next_pos)[0][0]
    #                                     # Referenced file: sk_utilities.py
    #                                     push_action = Push(box_num, abs_move.value)
    #                                     pos_behind_box = next_pos + self.game.step_fore[abs_move.value]
    #                                     # Referenced file: sk_game.py
    #                                     if self.logic._is_valid_and_clear(pos_behind_box, self.game.grid, self.game.boxes_index):
    #                                         self.game.move_box(push_action) # Referenced file: sk_game.py
    #                                         move_successful = True
    #                                         vis_title = f"FORWARD (Push {abs_move.name})"
    #                                     else: logging.error(f"Sim Error: Invalid push {abs_move.name} - blocked.")
    #                                 except IndexError: logging.error(f"Sim Error: Box index {next_pos} not found.")
    #                                 except Exception as e: logging.error(f"Sim Error during push: {e}")
    #                             else: # Simple Move
    #                                  # Referenced file: sk_game.py
    #                                 if self.game.move_robot(abs_move):
    #                                     move_successful = True
    #                                     vis_title = f"FORWARD (Move {abs_move.name})"
    #                                 else: logging.error(f"Sim Error: Failed move {abs_move.name}")

    #                             if not move_successful: simulation_ok = False; break
    #                             current_orientation = abs_move # Update orientation AFTER move
    #                             absolute_move_idx += 1
    #                         else:
    #                             logging.error("Sim Error: Ran out of absolute moves."); simulation_ok = False; break

    #                     elif relative_move == TURN_LEFT:
    #                         # Referenced file: sk_utilities.py
    #                         current_orientation = RobotDirection((current_orientation.value - 1 + 4) % 4)
    #                         vis_title = TURN_LEFT
    #                     elif relative_move == TURN_RIGHT:
    #                         # Referenced file: sk_utilities.py
    #                         current_orientation = RobotDirection((current_orientation.value + 1) % 4)
    #                         vis_title = TURN_RIGHT
    #                     elif relative_move == UTURN:
    #                         # Referenced file: sk_utilities.py
    #                         current_orientation = RobotDirection((current_orientation.value + 2) % 4)
    #                         vis_title = UTURN
    #                     else:
    #                         logging.warning(f"Unknown relative move: {relative_move}"); continue

    #                     # --- Visualize the result of the action ---
    #                     if not self.visualize_step(self.game, relative_step_count, vis_title, current_orientation): # Referenced file: sk_game.py
    #                         simulation_ok = False # Window closed during visualization
    #                         break

    #                     # --- Wait for user input to proceed ---
    #                     if not self.wait_for_step():
    #                         simulation_ok = False # User quit or closed window
    #                         break
    #                 # --- End of relative move loop ---

    #         except Exception as e:
    #              logging.error(f"An error occurred during visualization/simulation: {e}", exc_info=True)
    #              simulation_ok = False

    #         finally: # Ensure handler is disconnected and interactive mode turned off
    #             if 'cid' in locals() and cid is not None: # Disconnect only if connected
    #                 fig_vis.canvas.mpl_disconnect(cid)
    #                 logging.debug("Key press handler disconnected.")
    #             plt.ioff() # Turn off interactive mode AFTER simulation loop
    #             logging.debug("Matplotlib interactive mode OFF.")


    #         # --- Final state handling ---
    #         if simulation_ok:
    #             logging.info("\nFinal State (after simulation):")
    #             # Referenced file: sk_game.py
    #             if self.game.is_solved():
    #                 logging.info("Simulation Successful: Puzzle Solved!")
    #                 if plt.fignum_exists(fig_vis.number):
    #                      ax_vis.set_title(f"Relative Step {relative_step_count}: Final State - SOLVED!")
    #                      fig_vis.canvas.draw_idle() # Ensure final title is drawn
    #                      print("Puzzle solved! Close the plot window to exit.")
    #                      plt.show() # Keep final plot open
    #             else:
    #                 logging.error("Simulation Error: Final state is not solved.")
    #                 if plt.fignum_exists(fig_vis.number):
    #                      ax_vis.set_title(f"Relative Step {relative_step_count}: Final State - NOT SOLVED")
    #                      fig_vis.canvas.draw_idle()
    #                      print("Simulation ended but puzzle not solved. Close the plot window to exit.")
    #                      plt.show()
    #         elif _user_closed_window:
    #              logging.warning("Simulation aborted by user (quit key or window closed).")
    #              # Window might already be closed, but try closing just in case
    #              if plt.fignum_exists(fig_vis.number):
    #                   plt.close(fig_vis)
    #         else: # Simulation failed due to an error
    #              logging.error("Simulation failed due to an error.")
    #              if plt.fignum_exists(fig_vis.number):
    #                   ax_vis.set_title(f"Relative Step {relative_step_count}: SIMULATION ERROR")
    #                   fig_vis.canvas.draw_idle()
    #                   print("Simulation failed. Close the plot window to exit.")
    #                   plt.show() # Show error state

    #     else: # No solution found
    #          logging.info(f"No solution found within depth {self.sol_depth_max}")
    #          logging.info(f"Nodes expanded: {self.n_pushes_evaled}")
    #          logging.info(f"Nodes skipped: {self.n_pushes_skipped}")
    #          # No visualization to keep open if no solution


# --- Main Execution / Testing (remains the same) ---
if __name__ == "__main__":
    TEST_MAP_FILE = 'map_sk.csv'
    TEST_DEPTH = 50
    import os
    log_format = '%(asctime)s - %(levelname)s - %(message)s'
    # Add a file handler for detailed logs
    log_handlers = [logging.StreamHandler()] # Console handler
    log_file = 'sokoban_solver.log'
    try:
        # Use 'w' mode to overwrite log file each run
        file_handler = logging.FileHandler(log_file, mode='w')
        file_handler.setFormatter(logging.Formatter(log_format))
        log_handlers.append(file_handler)
    except Exception as e:
        print(f"Warning: Could not create log file handler for {log_file}: {e}")

    logging.basicConfig(level=logging.INFO, format=log_format, handlers=log_handlers)

    if os.path.exists(TEST_MAP_FILE):
        logging.info(f"--- Direct Solver Test: Running A* on {TEST_MAP_FILE} ---")
        try:
            solver = AStar(file_map_name=TEST_MAP_FILE, sol_depth_max=TEST_DEPTH)
            solver.solve(log=True)
        except FileNotFoundError:
             logging.error(f"Solver failed: Map file '{TEST_MAP_FILE}' not found during solver initialization.")
        except Exception as e:
             logging.error(f"An error occurred during solving: {e}", exc_info=True)
    else:
        logging.error(f"Direct Solver Test Error: Map file '{TEST_MAP_FILE}' not found.")
        print(f"Please ensure '{TEST_MAP_FILE}' exists in the same directory or provide the correct path.")

