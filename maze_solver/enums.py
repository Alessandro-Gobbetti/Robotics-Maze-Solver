from enum import Enum

class WallState(Enum):
    #(WEST, NORTH, EAST, SOUTH)
    #zero wall
    EMPTY = (0,0,0,0)
    #one wall
    WEST = (1,0,0,0)
    NORTH = (0,1,0,0)
    EAST = (0,0,1,0)
    SOUTH = (0,0,0,1)
    #two walls
    NORTH_WEST = (1,1,0,0)
    WEST_EAST = (1,0,1,0)
    SOUTH_WEST = (1,0,0,1)
    NORTH_EAST = (0,1,1,0)
    SOUTH_EAST = (0,0,1,1)
    NORTH_SOUTH = (0,1,0,1)
    #three walls
    WEST_NORTH_EAST = (1,1,1,0)
    WEST_EAST_SOUTH = (1,0,1,1)
    WEST_NORTH_SOUTH = (1,1,0,1)   
    NORTH_EAST_SOUTH = (0,1,1,1)
    #four walls
    WEST_NORTH_EAST_SOUTH = (1,1,1,1)

class FloodFillState(Enum):
    REACH_GOAL = 1
    REACH_START = 2
    SPRINT = 3

class CallbackUsage(Enum):
    CAMERA = 1
    ROT = 2
    CALC = 3
    MOVE_A_LITTLE_FORWARD = 4

class RotationVal(Enum):
    NINETY_LEFT = 1
    NINETY_RIGHT = 2
    BACK = 3

class Position(Enum):
    UP = 1
    RIGHT = 2
    LEFT = 3
    DOWN = 4
