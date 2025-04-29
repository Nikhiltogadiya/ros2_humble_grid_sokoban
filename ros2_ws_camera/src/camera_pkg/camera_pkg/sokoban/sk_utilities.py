from enum import Enum 

class Tile(Enum):
    BOX = 1<<0      # dec = 1
    FLOOR = 1<<1    # dec = 2
    GOAL = 1<<2     # dec = 4
    ROBOT = 1<<3    # dec = 8
    WALL = 1<<4     # dec = 16
    # Floor and Robot: 10
    # Floor and Box: 3
    # Floor and Goal: 6
    # Floor and Goal and Box: 7

class RobotDirection(Enum):
    UP = 0      
    DOWN = 1
    LEFT = 2
    RIGHT = 3

    def __str__(self):
        return self.name[:1]
    
class Push:
    def __init__(self, box_num, direction):
        self.box_num = box_num
        self.direction = direction



