# -*- coding: utf-8 -*-
import numpy as np
from .sk_game import SokobanGame, SokobanLogic # Add dot
from .sk_utilities import * # Add dot
# from sk_game import SokobanGame, SokobanLogic # Assuming SokobanLogic is now primarily in sk_game
# from sk_utilities import *
from collections import deque
import heapq
import logging
from typing import List, Tuple, Optional # Added for type hinting


class SearchAlgorithm:
    def __init__(self, file_map_name:str, sol_depth_max:int):
        self.file_map_name = file_map_name
        self.solved = False
        self.sol_depth_max = sol_depth_max
        self.count_depth = 0 # Represents number of pushes
        self.solution: List[Tuple[List[RobotDirection], Push]] = [] # Stores (robot_moves_before_push, push)
        self.final_robot_moves: List[RobotDirection] = [] # Flattened list of all robot moves

    def print_solver_info(self, log=False) -> None:
        log_function = logging.info if log else print
        log_function("\n=== Solver Information ===")
        # Avoid printing large objects like game grids or complex structures
        skip_keys = ['game', 'logic', 'open_list', 'closed_list', 'solution', 'final_robot_moves', 'nodes']
        for key, value in vars(self).items():
            if key not in skip_keys:
                 log_function(f"{key.replace('_', ' ').title()}: {value}")
        log_function("==========================\n")


class AStarNode:
    def __init__(self,
                 ztup: tuple[int, int],                  # (robot_index_after_push, state_zhash_after_push)
                 pred: Optional['AStarNode'],           # Predecessor node
                 robot_moves_before_push: Optional[List[RobotDirection]], # Moves to get into position for the push
                 push: Optional[Push],                  # The push action itself
                 push_depth: int,                       # Number of pushes (g cost)
                 lower_bound: int,                      # Heuristic cost (h cost)
                 game_state: tuple[int, np.ndarray]): # (robot_index_after_push, boxes_indices_after_push)

        self.ztup = ztup # State identifier (robot pos might differ for same box state)
        self.pred = pred
        self.robot_moves_before_push = robot_moves_before_push if robot_moves_before_push else []
        self.push = push # The push that led to this state
        self.push_depth = push_depth # g cost (number of pushes)
        self.lower_bound = lower_bound # h cost (heuristic)
        self.f = self.push_depth + self.lower_bound # f cost
        self.game_state = game_state # (robot_idx, boxes_indices) representing state *after* the push

        # --- Path Reconstruction ---
        # The 'path' stores the sequence of steps (robot moves + push) to reach this node's state from the start
        self.path: List[Tuple[List[RobotDirection], Push]] = []
        if pred is not None and self.push is not None:
            # Ensure robot_moves_before_push is treated as an empty list if None
            current_segment = (self.robot_moves_before_push, self.push)
            self.path = pred.path + [current_segment]


    def __lt__(self, other: "AStarNode") -> bool:
        # Prioritize lower f-cost. If f-costs are equal,
        # potentially prioritize lower h-cost (lower_bound) as a tie-breaker.
        if self.f != other.f:
            return self.f < other.f
        return self.lower_bound < other.lower_bound


class AStarHeuristic(SokobanLogic):
    def __init__(self, grid_shape: Tuple[int,int], wall_indices: np.ndarray, step_fore: np.ndarray, goal_indices: np.ndarray):
        super().__init__(grid_shape, wall_indices, step_fore) # Call parent constructor
        self.goal_index = goal_indices
        # Pass the grid *shape* and wall indices, not the grid state itself if logic is stateless
        self.grid_distance_to_goal = self._calc_dist_to_nearest_goal()
        self.simple_lower_bound = np.iinfo(np.int32).max # Initialize lower bound


    def _calc_dist_to_nearest_goal(self) -> np.ndarray:
        """ Calculates distance from each floor tile to the nearest goal using BFS. """
        grid_distance_to_goal = np.full(self.grid_size, np.iinfo(np.int32).max, dtype=np.int32)
        grid_distance_to_goal[self.wall_indices] = -1 # Mark walls explicitly if needed, or use MAX
        queue = deque()

        # Initialize goals
        for index in self.goal_index:
            if index not in self.wall_indices: # Should not happen, but check
                grid_distance_to_goal[index] = 0
                queue.append(index)

        # BFS from goals
        while queue:
            index = queue.popleft()
            current_dist = grid_distance_to_goal[index]

            for dir_enum in RobotDirection:
                neighbour_index = index + self.step_fore[dir_enum.value]

                if 0 <= neighbour_index < self.grid_size and grid_distance_to_goal[neighbour_index] == np.iinfo(np.int32).max:
                     # Check if neighbor is not a wall BEFORE updating distance
                     # Assuming walls are marked or initial grid state passed? Here use self.wall_indices
                     if neighbour_index not in self.wall_indices:
                          grid_distance_to_goal[neighbour_index] = current_dist + 1
                          queue.append(neighbour_index)
                     # else: neighbour is wall, distance remains MAX (or -1)

        # Mark unreachable floor tiles? If MAX remains, it's unreachable from any goal.
        # grid_distance_to_goal[grid_distance_to_goal == np.iinfo(np.int32).max] = -1 # Optional: mark unreachable non-walls
        return grid_distance_to_goal


    def calculate_simple_lower_bound(self, boxes_index: np.ndarray) -> int:
        """ Calculates the heuristic: sum of minimum distances of each box to its nearest goal. """
        bound = 0
        for box_idx in boxes_index:
            dist = self.grid_distance_to_goal[box_idx]
            if dist == np.iinfo(np.int32).max or dist == -1: # If box is on wall or unreachable tile
                self.simple_lower_bound = np.iinfo(np.int32).max # This state is unsolvable (dead end)
                return self.simple_lower_bound
            bound += dist

        # Simple matching - doesn't handle conflicts (multiple boxes needing same goal)
        self.simple_lower_bound = bound
        return bound

    def print_distance_grid(self):
        """ Prints the calculated distances to the nearest goal. """
        # Replace MAX or -1 with something readable like 'inf' or 'W'
        printable_dist = np.where(self.grid_distance_to_goal == np.iinfo(np.int32).max, -1, self.grid_distance_to_goal) # Use -1 for inf
        printable_dist = np.where(printable_dist == -1, 'inf', printable_dist).astype(str) # Replace -1 with 'inf' string

        max_len = max(len(s) for s in printable_dist)

        grid_2d = printable_dist.reshape(self.grid_rows, self.grid_cols)

        print("--- Distance to Nearest Goal Grid ---")
        for row in range(self.grid_rows):
            row_str = " ".join(item.rjust(max_len) for item in grid_2d[row])
            print(row_str)
        print("------------------------------------")


class OpenListAStar:
    def __init__(self) -> None:
        self.state_list = {} # Stores {ztup: lowest_push_depth_seen}
        self.heap = [] # Stores (f_cost, node_instance)
        self._counter = 0 # Tie-breaker for heap ordering

    def add_node(self, node: AStarNode) -> None:
        """ Adds a node to the open list, replacing if a better path is found. """
        # Check if state exists and if the new node is worse or equal
        if node.ztup in self.state_list and node.push_depth >= self.state_list[node.ztup]:
            return  # Ignore worse or equivalent push depth nodes

        # Add or update the node
        self.state_list[node.ztup] = node.push_depth
        heapq.heappush(self.heap, (node.f, node.lower_bound, self._counter, node)) # Use f, then h, then counter for tie-breaking
        self._counter += 1


    def pop_node(self) -> AStarNode | None:
        """ Pops the node with the lowest f-cost from the open list. """
        while self.heap:
            f_cost, h_cost, counter, node = heapq.heappop(self.heap)
            # Crucial Check: Ensure the popped node corresponds to the *best known* path
            # to its state (ztup). If we found a shorter path later and pushed it,
            # the state_list[node.ztup] would be lower than node.push_depth.
            if node.push_depth <= self.state_list[node.ztup]:
                 # This is the best path found so far for this state (or equal), process it.
                 # We can remove it from state_list here or rely on closed list check later.
                 # del self.state_list[node.ztup] # Optional: remove from tracking once popped as 'best'
                 return node
            # else: This node is outdated (a better path was found later), discard it.
        return None  # Heap is empty


    def is_empty(self) -> bool:
        return len(self.heap) == 0

# --- Testing Functions ---
def testing_a_star_heuristic() -> None:
    # Requires a map file, e.g., 'map_sk.csv' generated by main.py/sk_printmap.py
    try:
        sg = SokobanGame('map_sk.csv')
    except FileNotFoundError:
        print("Error: map_sk.csv not found. Run main.py first to generate it.")
        return

    print("--- Initial Game State ---")
    sg.print_grid(log=False)
    sg.print_grid_with_indices()

    print("\n--- Initializing A* Heuristic ---")
    ash = AStarHeuristic(
        grid_shape=(sg.grid_rows, sg.grid_cols),
        wall_indices=sg.wall_index,
        step_fore=sg.step_fore,
        goal_indices=sg.goals_index
    )

    print("\n--- Calculating Distances to Goals ---")
    ash.print_distance_grid()

    print("\n--- Calculating Initial Lower Bound ---")
    initial_lb = ash.calculate_simple_lower_bound(sg.boxes_index)
    print(f"Initial simple lower bound (heuristic): {initial_lb}")

    print("\n--- Testing Logic Functions (inherited) ---")
    # Test pathfinding inherited from SokobanLogic
    print("Finding path from robot start to a goal (if possible):")
    target_goal = sg.goals_index[0] if len(sg.goals_index) > 0 else -1
    if target_goal != -1:
         path = ash.find_robot_path(sg.grid, sg.robot_index, target_goal, sg.boxes_index)
         if path:
             print(f" Path to goal {target_goal}: {[d.name for d in path]}")
         else:
             print(f" Path to goal {target_goal} not found or blocked.")

if __name__ == "__main__":
    testing_a_star_heuristic()