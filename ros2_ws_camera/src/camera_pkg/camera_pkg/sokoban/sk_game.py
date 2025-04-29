# -*- coding: utf-8 -*-
import numpy as np                  # NumPy library for calculation
# from sk_utilities import *
from .sk_utilities import * # Add the dot before sk_utilities
import pandas as pd                 # To read map from csv file
from collections import deque
import logging
from typing import List, Tuple, Optional # Added for type hinting

class SokobanGame:
    def __init__(self) -> None: # <--- CORRECTED
    # def __init__(self, file_name:str) -> None:
        # self.file_name = file_name
        
        # --- ADD: Define your map layout here ---
        # Replace this with your actual map. Use Tile enum values.
        # Example: 5x5 map with walls, floor, 1 robot, 1 box, 1 goal
        # WALL=16, FLOOR=2, ROBOT=8, BOX=1, GOAL=4
        # ROBOT+FLOOR = 10, BOX+FLOOR = 3, GOAL+FLOOR = 6
        
        hardcoded_map_data = [
            [16, 16, 16, 16, 16, 16, 16],  # Row 0: Top Wall
            [16,  10,  2,  2,  2,  2, 16],  # Row 1: Floor
            [16,  2,  2,  2,  2,  6, 16],  # Row 2: Floor, Robot, Box, Goal, Floor
            [16,  2,  2,  2,  3,  2, 16],  # Row 3: Floor
            [16,  2,  2,  16,  16,  2, 16],  # Row 4: Floor
            [16,  2,  3,  6,  16,  16, 16],  # Row 5: Floor
            [16, 16, 16, 16, 16, 16, 16]   # Row 6: Bottom Wall

        ]
            # 1
            # [16, 16, 16, 16, 16],
            # [16, 10,  2,  3, 16], # Robot on Floor, Floor, Box on Floor
            # [16,  2,  2,  2, 16],
            # [16,  2,  6,  2, 16], # Floor, Goal on Floor, Floor
            # [16, 16, 16, 16, 16]
            # original from external script
            # [16,16,16,16,16,16,16,16,16],
            # [16,10,2,2,2,2,2,2,16],
            # [16,2,2,2,2,2,2,2,16],
            # [16,2,3,3,3,6,6,6,16],
            # [16,2,2,2,2,16,16,16,16],
            # [16,2,2,2,2,16,16,16,16],
            # [16,16,16,16,16,16,16,16,16]
            # 2
            # [16, 16, 16, 16, 16, 16],
            # [16,  2,  1,  2,  4, 16],
            # [16,  2, 16,  2,  2, 16],
            # [16,  2,  2,  2,  2, 16],
            # [16,  8,  2,  1,  4, 16],
            # [16, 16, 16, 16, 16, 16],
            # 3
            # [16,16,16,16,16,16,16,16,16],
            # [16,10, 2, 2, 2, 2, 2, 2,16], # R . . . . . . . #
            # [16, 2, 2, 2, 2, 2, 2, 2,16], # . . . . . . . . #
            # [16, 2, 3, 3, 3, 6, 6, 6,16], # . . B B B G G G #
            # [16, 2, 2, 2, 2,16,16,16,16], # . . . . # # # # #
            # [16, 2, 2, 2, 2,16,16,16,16], # . . . . # # # # #
            # [16,16,16,16,16,16,16,16,16]
        # --- END: Map Definition ---
        
        # Ensure dtype is appropriate, e.g., int or potentially uint8 if values are small
        # self.grid_base = np.array(pd.read_csv(self.file_name, header=None).values, dtype=np.int32)
        self.grid_base = np.array(hardcoded_map_data, dtype=np.int32)
        self.grid_rows, self.grid_cols = self.grid_base.shape
        self.grid_base_flat = self.grid_base.flatten()
        self.grid = self.grid_base_flat.copy() # Working copy
        self.hash_components = np.random.randint(0, 2**64, size=len(self.grid), dtype=np.uint64)
        self.step_fore = np.array([-self.grid_cols, self.grid_cols, -1, 1], dtype=np.int32) # Up, Down, Left, Right
        self.goals_index = np.where(self.grid_base_flat & Tile.GOAL.value != 0)[0]
        self.wall_index = np.where(self.grid_base_flat & Tile.WALL.value != 0)[0]
        self.reset()

    def reset(self) -> None:
        self.grid = self.grid_base_flat.copy() # Reset to initial state
        self.robot_index = int(np.where(self.grid & Tile.ROBOT.value != 0)[0][0]) # Ensure single int
        self.boxes_index = np.where(self.grid & Tile.BOX.value != 0)[0]
        # Ensure goals_index and wall_index are set if not in __init__
        # self.goals_index = np.where(self.grid_base_flat & Tile.GOAL.value != 0)[0]
        # self.wall_index = np.where(self.grid_base_flat & Tile.WALL.value != 0)[0]
        self.state_zhash = np.uint64(0)
        for box_index in self.boxes_index:
            self.state_zhash ^= self.hash_components[box_index]

    def is_solved(self) -> bool:
        # Check if all box positions are also goal positions
        current_boxes_on_goals = np.sum(np.isin(self.boxes_index, self.goals_index))
        return current_boxes_on_goals == len(self.goals_index)


    def move_robot(self, dir:RobotDirection):
        """ Moves only the robot without pushing. """
        next_robot_index = self.robot_index + self.step_fore[dir.value]
        # Check bounds and if the next tile is floor/goal (not wall or box)
        if 0 <= next_robot_index < len(self.grid) and \
           not (self.grid[next_robot_index] & Tile.WALL.value) and \
           not (self.grid[next_robot_index] & Tile.BOX.value):
            self.grid[self.robot_index] &= ~Tile.ROBOT.value # Remove robot from old pos
            self.robot_index = next_robot_index
            self.grid[self.robot_index] |= Tile.ROBOT.value # Place robot in new pos
            return True
        return False


    def move_box(self, push:Push) -> None:
        # Calculate index
        box_current_index = self.boxes_index[push.box_num]
        box_next_index = box_current_index + self.step_fore[push.direction]
        robot_current_index = self.robot_index # Should be box_current_index - step_fore[push.direction]
        robot_next_index = box_current_index # Robot moves into the box's old spot

        # Update the grid
        self.grid[robot_current_index] &= ~Tile.ROBOT.value # Remove robot from its pos behind box
        self.grid[box_current_index] &= ~Tile.BOX.value     # Remove box from its old pos
        self.grid[box_current_index] |= Tile.ROBOT.value    # Move robot to box's old pos
        self.grid[box_next_index] |= Tile.BOX.value         # Move box to its new pos

        # Update internal state
        self.robot_index = robot_next_index
        self.boxes_index[push.box_num] = box_next_index

        # Update Zobrist hash
        self.state_zhash ^= self.hash_components[box_current_index]
        self.state_zhash ^= self.hash_components[box_next_index]


    def unmove_box(self, push:Push) -> None:
        # Calculate index based on the state *after* the push
        box_new_index = self.boxes_index[push.box_num] # Box's current position (after push)
        box_original_index = box_new_index - self.step_fore[push.direction] # Box's position before push
        robot_original_index = box_original_index - self.step_fore[push.direction] # Robot's position before push
        robot_current_index = box_original_index # Robot's current position (after push, was box's original pos)


        # Update the grid (reverse of move_box)
        self.grid[box_new_index] &= ~Tile.BOX.value         # Remove box from its new pos
        self.grid[robot_current_index] &= ~Tile.ROBOT.value # Remove robot from its current pos (box's original pos)
        self.grid[box_original_index] |= Tile.BOX.value     # Restore box to its original pos
        self.grid[robot_original_index] |= Tile.ROBOT.value # Restore robot to its original pos


        # Update internal state
        self.robot_index = robot_original_index
        self.boxes_index[push.box_num] = box_original_index

        # Update Zobrist hash (reverse)
        self.state_zhash ^= self.hash_components[box_new_index]
        self.state_zhash ^= self.hash_components[box_original_index]


    def set_robot_index(self, index:int) -> None:
        """ Force sets the robot index, ensuring grid consistency. """
        if self.robot_index is not None: # Ensure robot_index has been initialized
             self.grid[self.robot_index] &= ~Tile.ROBOT.value # Remove from old position if exists
        self.robot_index = index
        self.grid[self.robot_index] |= Tile.ROBOT.value # Add to new position


    def set_game_state(self, robot_index: int, boxes_index: np.ndarray) -> None:
        """ Sets the entire game state from robot and boxes indices. """
        # Clear current robot and boxes from grid
        if hasattr(self, 'robot_index') and self.robot_index is not None:
            self.grid[self.robot_index] &= ~Tile.ROBOT.value
        if hasattr(self, 'boxes_index'):
            self.grid[self.boxes_index] &= ~Tile.BOX.value

        # Restore underlying floor/goal tiles
        self.grid = self.grid_base_flat.copy()

        # Set new state
        self.robot_index = robot_index
        self.boxes_index = boxes_index.copy() # Use copy to avoid modifying source array

        # Place new robot and boxes on grid
        self.grid[self.robot_index] |= Tile.ROBOT.value
        self.grid[self.boxes_index] |= Tile.BOX.value

        # Recalculate Zobrist hash
        self.state_zhash = np.uint64(0)
        for box_idx in self.boxes_index:
            self.state_zhash ^= self.hash_components[box_idx]


    def print_grid(self, log = False) -> None:
        tile_ascii = {
            # Base tiles
            Tile.WALL.value: "#",
            Tile.FLOOR.value: " ", # Use space for floor for clarity? Or "."
            Tile.GOAL.value: ".", # Use . for goal

            # Combined tiles
            (Tile.FLOOR.value | Tile.ROBOT.value): "@",
            (Tile.FLOOR.value | Tile.BOX.value): "$",
            (Tile.GOAL.value | Tile.BOX.value): "*",   # Box on goal
            (Tile.GOAL.value | Tile.ROBOT.value): "+", # Robot on goal
            # (Tile.FLOOR.value | Tile.GOAL.value) is just Tile.GOAL.value if goal always implies floor underneath
            # Need to handle potential BOX+ROBOT+GOAL? (Shouldn't happen if robot pushes)

            # Handle cases where the base map might use different codes than expected
            16: "#", # Explicitly map wall code if needed
            2: " ", # Explicitly map floor code if needed
            6: ".", # Explicitly map goal code if needed
             # Add other direct mappings if the input CSV uses them directly
             # instead of bitwise combinations initially
            3: "$", # floor+box
            10: "@", # floor+robot
            7: "*", # floor+box+goal

        }
        log_function = logging.info if log else print
        grid_2d = self.grid.reshape(self.grid_rows, self.grid_cols)
        log_function("-" * self.grid_cols) # Separator
        for row in range(self.grid_rows):
            row_str = ""
            for col in range(self.grid_cols):
                 tile_val = int(grid_2d[row, col])
                 # Simplify logic: check components
                 is_wall = bool(tile_val & Tile.WALL.value)
                 is_box = bool(tile_val & Tile.BOX.value)
                 is_robot = bool(tile_val & Tile.ROBOT.value)
                 is_goal = bool(tile_val & Tile.GOAL.value)

                 char = "?" # Default for unknown
                 if is_wall:
                     char = "#"
                 elif is_box:
                     char = "*" if is_goal else "$"
                 elif is_robot:
                     char = "+" if is_goal else "@"
                 elif is_goal:
                     char = "."
                 else: # Must be floor
                     char = " "

                 row_str += char
                 # Old logic - might be less robust if multiple things occupy a tile unexpectedly
                 # row_str += tile_ascii.get(int(grid_2d[row, col]), "?")
            log_function(row_str)
        log_function("-" * self.grid_cols) # Separator


    def print_grid_with_indices(self) -> None:
        max_index = self.grid_rows * self.grid_cols - 1
        max_index_length = len(str(max_index))
        index = 0
        for row in range(self.grid_rows):
            index_row = []
            for col in range(self.grid_cols):
                index_row.append(str(index).rjust(max_index_length))
                index += 1
            print(" ".join(index_row))


class SokobanLogic:
    def __init__(self, grid_shape: Tuple[int, int], wall_indices: np.ndarray, step_fore: np.ndarray) -> None:
        self.grid_rows, self.grid_cols = grid_shape
        self.grid_size = self.grid_rows * self.grid_cols
        self.wall_indices = wall_indices
        self.step_fore = step_fore # [-cols, +cols, -1, +1]

        # For reachability - persistent state across calls if needed, or reset per call
        self.tiles_visited = np.zeros(self.grid_size, dtype=np.uint32)
        self.visit_counter = 0 # Increment this for each new reachability search
        self.min_reachable_tile = -1 # Reset per calculation


    def _is_valid_and_clear(self, index: int, current_grid: np.ndarray, boxes_indices: np.ndarray) -> bool:
        """ Checks if an index is within bounds, not a wall, and not a box. """
        if not (0 <= index < self.grid_size):
            return False
        if current_grid[index] & Tile.WALL.value:
            return False
        if index in boxes_indices: # More robust check using current box locations
            return False
        return True

    def find_robot_path(self, current_grid: np.ndarray, start_index: int, end_index: int, boxes_indices: np.ndarray) -> Optional[List[RobotDirection]]:
        """ Finds the shortest path for the robot using BFS. """
        if start_index == end_index:
            return []

        q = deque([(start_index, [])]) # Store (index, path_list)
        visited = {start_index}

        while q:
            current_index, path = q.popleft()

            for direction in RobotDirection:
                next_index = current_index + self.step_fore[direction.value]

                if next_index == end_index:
                    return path + [direction] # Found the path

                if self._is_valid_and_clear(next_index, current_grid, boxes_indices) and next_index not in visited:
                    visited.add(next_index)
                    new_path = path + [direction]
                    q.append((next_index, new_path))

        return None # Target not reachable


    def calc_reachable_tiles_and_potential_pushes(self, grid: np.ndarray, start_index: int, boxes_index: np.ndarray) -> Tuple[set, dict]:
        """
        Calculates tiles reachable by the robot and identifies potential push locations.
        Returns:
            - set: Indices of all tiles reachable by the robot.
            - dict: {box_index: {direction: robot_pos_for_push}} identifying possible pushes.
        """
        start_index = int(start_index)
        to_visit = deque([start_index])
        reachable_tiles = {start_index}
        potential_pushes = {} # {box_idx: {RobotDirection: robot_pos}}

        self.min_reachable_tile = start_index

        while to_visit:
            index = to_visit.popleft()
            self.min_reachable_tile = min(self.min_reachable_tile, index)

            for dir_enum in RobotDirection:
                direction_val = dir_enum.value
                neighbour = index + self.step_fore[direction_val]

                if not (0 <= neighbour < self.grid_size):
                    continue # Skip out-of-bounds

                is_wall = bool(grid[neighbour] & Tile.WALL.value)
                is_box = neighbour in boxes_index

                if not is_wall:
                    if not is_box: # It's a floor/goal tile
                        if neighbour not in reachable_tiles:
                            reachable_tiles.add(neighbour)
                            to_visit.append(neighbour)
                    else: # It's a box
                        # Can the robot potentially push this box? Check space behind box
                        next_box_index = neighbour + self.step_fore[direction_val]
                        if self._is_valid_and_clear(next_box_index, grid, boxes_index):
                            # Robot at 'index' can potentially push box at 'neighbour' in direction 'dir_enum'
                            if neighbour not in potential_pushes:
                                potential_pushes[neighbour] = {}
                            potential_pushes[neighbour][dir_enum] = index # Robot needs to be at 'index'

        return reachable_tiles, potential_pushes


    def calc_available_pushes(self, grid: np.ndarray, start_index: int, current_boxes_index: np.ndarray) -> List[Tuple[Push, int]]:
        """
        Calculates available pushes based on robot reachability.
        Returns a list of tuples: (Push object, robot_position_needed_for_push)
        """
        # Find where the robot can reach
        reachable_tiles, potential_pushes = self.calc_reachable_tiles_and_potential_pushes(grid, start_index, current_boxes_index)

        available_pushes = []
        box_index_map = {box_idx: i for i, box_idx in enumerate(current_boxes_index)} # Map box index to its number/pos in array

        for box_idx, pushes_for_box in potential_pushes.items():
            for direction, robot_pos in pushes_for_box.items():
                # Check if the position the robot needs to be in ('robot_pos') is actually reachable
                if robot_pos in reachable_tiles:
                    if box_idx in box_index_map: # Ensure the box is currently tracked
                         box_num = box_index_map[box_idx]
                         available_pushes.append((Push(box_num, direction.value), robot_pos))
                    # else: log warning maybe? Box detected but not in current_boxes_index

        # Set the minimal reachable tile based on this calculation
        # self.set_robot_index(self.min_reachable_tile) # This shouldn't be done here, logic modifies game state

        return available_pushes


# --- Testing Functions ---
def testing_sokoban_game() -> None:
    sg = SokobanGame('map_sk.csv') # Use the output file from print_map
    sg.print_grid(True)
    sg.print_grid_with_indices()
    print("Initial Robot Index:", sg.robot_index)
    print("Initial Boxes Index:", sg.boxes_index)
    print("Goal Indices:", sg.goals_index)
    # print("Grid:", sg.grid)

def testing_sokoban_logic() -> None:
    sg = SokobanGame('map_sk.csv')
    sl = SokobanLogic(
        grid_shape=(sg.grid_rows, sg.grid_cols),
        wall_indices=sg.wall_index,
        step_fore=sg.step_fore
    )

    print("--- Testing Reachability and Pushes ---")
    reachable, potentials = sl.calc_reachable_tiles_and_potential_pushes(sg.grid, sg.robot_index, sg.boxes_index)
    print(f"Robot can reach {len(reachable)} tiles.")
    # print(f"Reachable tiles: {sorted(list(reachable))}")
    print(f"Potential pushes (box_idx: {{direction: robot_pos}}): {potentials}")
    print(f"Min reachable tile index: {sl.min_reachable_tile}")

    print("\n--- Testing Available Pushes ---")
    # Need the game state (grid, robot_pos, boxes_pos)
    pushes_with_robot_pos = sl.calc_available_pushes(sg.grid, sg.robot_index, sg.boxes_index)
    print(f"Found {len(pushes_with_robot_pos)} available pushes:")
    for push, robot_pos in pushes_with_robot_pos:
        print(f"  Push Box {push.box_num} (at {sg.boxes_index[push.box_num]}) Dir {RobotDirection(push.direction)} requires robot at {robot_pos}")

    print("\n--- Testing Robot Pathfinding ---")
    # Example: Find path from current robot pos to a reachable floor tile
    target_tile = -1
    for tile in reachable:
        if tile != sg.robot_index:
            target_tile = tile
            break

    if target_tile != -1:
        print(f"Finding path from {sg.robot_index} to {target_tile}")
        path = sl.find_robot_path(sg.grid, sg.robot_index, target_tile, sg.boxes_index)
        if path:
            print(f"  Path found: {[d.name for d in path]}")
        else:
            print(f"  Path not found (should not happen if target is reachable).")
    else:
        print("Could not find a different reachable tile to test pathfinding.")

    # Example: Find path needed for the first available push
    if pushes_with_robot_pos:
        first_push, robot_needed_pos = pushes_with_robot_pos[0]
        print(f"\nFinding path for first push (Box {first_push.box_num} Dir {RobotDirection(first_push.direction)}) from {sg.robot_index} to {robot_needed_pos}")
        path_to_push = sl.find_robot_path(sg.grid, sg.robot_index, robot_needed_pos, sg.boxes_index)
        if path_to_push is not None: # Path can be empty list if already there
            print(f"  Path found: {[d.name for d in path_to_push]}")
        else:
            print(f"  Path not found.")


if __name__ == "__main__":
    # Make sure map_sk.csv exists (run main.py once first)
    # testing_sokoban_game()
    testing_sokoban_logic()