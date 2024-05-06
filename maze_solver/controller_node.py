import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from collections import deque
from enum import Enum
import sys

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, LaserScan, Imu
from math import pi


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

class ThymioState(Enum):
    DETECT = 1
    DECIDE = 2
    WALK = 3
    DONE = 4

class MovingState(Enum):
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.odom_pose = None
        self.odom_velocity = None
        self.pose2d = None
        self.prev_cell_odom_pose = None
        self.start_pose = None
        self.new_pose = None
        # self.wall_state_robot = None
        self.wall_state_global = None

        self.mode = ThymioState.WALK
        self.floodfill_state = FloodFillState.REACH_GOAL
        self.explored = []
        self.best_paths = []
        self.best_path = None
        self.next_cell = None
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        

        self.initial_rotation = -pi/2
        self.dim = (5,5)
        self.goal = (2,2)
        self.start = (4, 0)
        self.flood_matrix = self.create_flood_matrix(self.dim, self.goal)
       
        # the maze array is a 2d array: each entry is a list of neighbors
        self.maze_matrix = self.create_maze_matrix(self.dim)
        


        

        self.cell_side_length = 0.25 # Defines the length of the side of the cell
        # self.cell_area = self.cell_side_length ** 2
        # self.start_vel = 0.1 # Defines the start velocity of the cell
        # self.maze_side = 5.0 # Defines the length of the side of the maze
        # self.maze_area = 5.0**2 # The area of the maze that we are traversing
        # self.num_cells = self.maze_side // self.cell_side_length
        # self.maze_matrix = np.zeros((self.num_cells, self.num_cells)) # Creates the matrix by dividing the maze into cells that are defined as above. The elements denote how far we are from the goal.
        # self.start_cell = (0,0) # We start in the middle of the cell
        # self.dist_from_start = 0.0 # Defines how many cells away we are from the starting cell
        self.distance_tolerance = 0.001
        self.current_cell = (0,0)
        self.move_dir = MovingState.DOWN
        self.proximities = {
            'left': -1,
            'center_left': -1,
            'center': -1,
            'center_right': -1,
            'right': -1,
            'right_back': -1,
            'left_back': -1
        }
        # Defining the proximity subscribers:
        self.prox_left = self.create_subscription(Range, 'proximity/left', lambda message: self.proximity_callback(message, "left"), 10)
        self.prox_right = self.create_subscription(Range, 'proximity/right', lambda message: self.proximity_callback(message, "right"), 10)
        self.prox_center = self.create_subscription(Range, 'proximity/center', lambda message: self.proximity_callback(message, "center"), 10)
        self.prox_center_left = self.create_subscription(Range, 'proximity/center_left', lambda message: self.proximity_callback(message, "center_left"), 10)
        self.prox_center_right = self.create_subscription(Range, 'proximity/center_right', lambda message: self.proximity_callback(message, "center_right"), 10)
        self.prox_left_back= self.create_subscription(Range, 'proximity/rear_left', lambda message: self.proximity_callback(message, "left_back"), 10)
        self.prox_right_back = self.create_subscription(Range, 'proximity/rear_right', lambda message: self.proximity_callback(message, "right_back"), 10)
        
    
    def run(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    # --------------------------------------------------------------------------------------------
    # main loop

    def update_callback(self):

        if self.next_cell:
            self.goto(self.next_cell)


        # if self.mode == ThymioState.DETECT:
        #     self.update_detect()
        # elif self.mode == ThymioState.DECIDE:
        #     self.update_decide()
        # elif self.mode == ThymioState.WALK:
        # # Let's just set some hard-coded velocities in this example
        #     cmd_vel = Twist() 
        #     cmd_vel.linear.x  = 0.2 # [m/s]
        #     cmd_vel.angular.z = 0.0 # [rad/s]
        #     # Publish the command
        #     self.vel_publisher.publish(cmd_vel)
        # elif self.mode == ThymioState.DONE:
        #     return 

    def goto(self, cell_coord):

        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.1
        # cmd_vel.angular.z = 0.0
        # self.vel_publisher.publish(cmd_vel)

        current_odom_pose = self.new_pose
        current_cell_coord = self.current_cell
        target_x = cell_coord[0] - current_cell_coord[0]
        target_y = cell_coord[1] - current_cell_coord[1]

        if target_x > 0:
            # We move to the left
            pass
        elif target_x < 0:
            # We move to the right
            pass
        elif target_y > 0:
            # We move down
            pass
        else:
            pass
            # We move up


    # --------------------------------------------------------------------------------------------
    # Callback functions

    def proximity_callback(self, message, which):
        self.proximities[which] = message.range # Updating the dict values with the current proximity value 
    
    def odom_callback(self, msg):
        # TODO: maybe we should split thise into multiple functions, the odom callback should just update the pose and velocity (and maybe the current cell) ?
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        pose2d = self.pose3d_to_2d(self.odom_pose)
        self.get_logger().info(f"Got Pose {pose2d}")
        self.new_pose = (pose2d[0], pose2d[1], pose2d[2])
        if self.start_pose is None:
            self.start_pose = (pose2d[0], pose2d[1], pose2d[2])
            self.prev_cell_odom_pose = (pose2d[0], pose2d[1], pose2d[2])
            # self.maze_matrix[self.start_cell] = self.dist_from_start
        
        current_pose = np.array([self.new_pose[0], self.new_pose[1]])
        prev_pose = np.array([self.prev_cell_odom_pose[0], self.prev_cell_odom_pose[1]])
        if np.abs(np.linalg.norm(current_pose - prev_pose) - self.cell_side_length) > self.distance_tolerance:
            #FIXME: do not enter if still on the same cell
            # We are in the center of a cell: update values and call step on the floodfill algorithm
            match self.move_dir:
                case MovingState.LEFT:
                    self.current_cell = (self.current_cell[0] + 1, self.current_cell[1])
                    
                case MovingState.RIGHT:
                    self.current_cell = (self.current_cell[0]-1, self.current_cell[1])
                    
                case MovingState.UP:
                    self.current_cell = (self.current_cell[0], self.current_cell[1] - 1)
                    
                case MovingState.DOWN:
                    self.current_cell = (self.current_cell[0], self.current_cell[1] + 1)

                case _:
                    self.get_logger().info("No Valid Case Matched.")

            self.get_logger().info(f"Current Cell: {self.current_cell}")
            
            # FIXME
            self.current_cell = self.start
            # if self.moved_left:
            #     self.current_cell = (self.current_cell[0] + 1, self.current_cell[1])
            # elif self.moved_right:
            #     self.current_cell = (self.current_cell[0]-1, self.current_cell[1])
            # elif self.up:
            #     self.current_cell = (self.current_cell[0], self.current_cell[1] - 1)
            # else:
            #     self.current_cell = (self.current_cell[0], self.current_cell[1] + 1)
            # if self.maze_matrix[self.current_cell] > 0.0:
            #     self.dist_from_start -= 1
            # else:
            #     self.dist_from_start += 1
            #     self.maze_matrix[self.current_cell] = self.dist_from_start

            # Detect walls and compute the next cell
            if self.floodfill_state == FloodFillState.REACH_GOAL or self.floodfill_state == FloodFillState.REACH_START:
                self.detect_walls()
                self.next_cell = self.floodfill_step()

    


            # TODO: move this to another function

            if self.floodfill_state == FloodFillState.REACH_GOAL and self.current_cell == self.goal :
                # compute the shortest path
                shortest_path, length = self.find_shortest_path(self.explored)
                self.best_paths.append(shortest_path)
                self.get_logger().info(f"Shortest Path: {shortest_path} with length {length}")

                # invert the flood matrix to reach the start
                self.flood_matrix = self.create_flood_matrix(self.dim, self.start)

                self.floodfill_state = FloodFillState.REACH_START
                self.logger.info("Goal reached. Returning to start.")

            if self.floodfill_state == FloodFillState.REACH_START and self.current_cell == self.start:
                # extract the path from goal to start form the explored list
                # find the goal node and split the list
                goal_index = self.explored.index(self.goal)
                explored = self.explored[:goal_index+1]                

                # compute the shortest path
                shortest_path, length = self.find_shortest_path(explored.reverse())
                self.best_paths.append(shortest_path)
                self.get_logger().info(f"Shortest Path: {shortest_path} with length {length}")
                
                if len(self.best_paths) > 1:
                    self.get_logger().info("Multiple paths found. Deciding the best path...")
                    self.best_path = min(self.best_paths, key=lambda x: len(x))
                else:
                    self.best_path = self.best_paths[0]

                
                self.floodfill_state = FloodFillState.SPRINT
                self.get_logger().info("Sprint Mode Activated.")
                self.get_logger().info(f"Best Path: {self.best_path}, Length: {len(self.best_path)}")

            if self.floodfill_state == FloodFillState.SPRINT and self.current_cell == self.goal:
                self.get_logger().info("Goal reached. Done.")
                self.mode = ThymioState.DONE

                


            self.prev_cell_odom_pose = (self.new_pose[0], self.new_pose[1], self.new_pose[2])
            
        #self.get_logger().info(
        #    "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        #)
    
    # --------------------------------------------------------------------------------------------
    # wall detection and decision making
    def detect_walls(self):

        walls_robot = WallState((int(self.proximities['left']>0), int(self.proximities['center']>0), int(self.proximities['right']>0), int(self.proximities['left_back']>0 and self.proximities['right_back']>0)))
        walls = WallState(self.rotate(walls_robot.value, self.initial_rotation))
        self.wall_state_global = WallState(self.rotate(walls.value, self.new_pose[2]))
        
        old_neighbors = self.maze_matrix[self.current_cell[0]][self.current_cell[1]]
        new_neighbors = [n if not w else None for n, w in zip(self.maze_matrix[self.current_cell[0]][self.current_cell[1]], walls.value)]
        # remove current cell as neighbor for all the removed neighbors
        for i, n in enumerate(old_neighbors):
            if n is not None and new_neighbors[i] is None:
                self.maze_matrix[n[0]][n[1]] = [x if x != self.current_cell else None for x in self.maze_matrix[n[0]][n[1]]]

        self.maze_matrix[self.current_cell[0]][self.current_cell[1]] = new_neighbors
        
        self.print_maze()
        
        self.get_logger().info(f"Walls : {walls_robot.value},\nRotated Walls: {walls.value}, \nGlobal Wall State: {self.wall_state_global.value}")


    def rotate(self, vec, angle_pi):
        angle_pi = round(angle_pi/(pi/2)) % 4
        return vec[angle_pi:] + vec[:angle_pi]
        


    # TODO: floodfill algorithm

    # --------------------------------------------------------------------------------------------
    # Helper functions
    def create_flood_matrix(self, dim, goal):
        flood_mat = np.zeros(dim, dtype=int)
        # each cell is filled with the distance to the goal
        for i in range(dim[0]):
            for j in range(dim[1]):
                flood_mat[i][j] = abs(i - goal[0]) + abs(j - goal[1])
        return flood_mat
    
    def create_maze_matrix(self, dim):
        maze_mat= np.zeros(dim, dtype=object)
        for i in range(dim[0]):
            for j in range(dim[1]):
                neighbors = [(i + dx, j + dy) 
                             if (0 <= i + dx < dim[0] and 0 <= j + dy < dim[1]) else None                             
                             for dx, dy in [(0,-1), (-1,0), (0, 1), (1, 0)]
                             ]
                maze_mat[i][j] = neighbors
        return maze_mat

    def get_min_neighbors(self, neighbors, flood_array):
        print("neighbors: ", neighbors)
        min_value = min([flood_array[n[0]][n[1]] for n in neighbors if n])
        return [n for n in neighbors if n and flood_array[n[0]][n[1]] == min_value], min_value




    def floodfill_step(self):
        """
        @return: the next cell to move to
        """
        neighbors = self.maze_matrix[self.current_cell[0]][self.current_cell[1]]
        queue = deque()

        # get all the neighbors with the smallest value
        smallest_neighbors, smallest_neighbor_value = self.get_min_neighbors(neighbors, self.flood_matrix)

        # if current smaller than all neighbors, floodfill the neighbors
        if self.flood_matrix[self.current_cell[0]][self.current_cell[1]] < smallest_neighbor_value:
            queue.append(self.current_cell)

            while not len(queue) == 0:
                el = queue.popleft()
                el_neighbors = self.maze_matrix[el[0]][el[1]]
                print("el: ", el, el_neighbors)
                el_smallest_neighbor = min(el_neighbors, key=lambda x: self.flood_matrix[x[0]][x[1]])
                el_smallest_neighbor_value = self.flood_matrix[el_smallest_neighbor[0]][el_smallest_neighbor[1]]
                # if el smaller than smallest neighbor
                if self.flood_matrix[el[0]][el[1]] <= el_smallest_neighbor_value:
                    self.flood_matrix[el[0]][el[1]] = el_smallest_neighbor_value + 1
                    for n in el_neighbors:
                        queue.append(n)
            
            # recompute the smallest neighbors
            smallest_neighbors, smallest_neighbor_value = self.get_min_neighbors(neighbors, self.flood_matrix)

        # pick one of the neighbors, prefer the one that is not in the explored list
        smallest_neighbor = None
        for n in smallest_neighbors:
            if n not in self.explored:
                smallest_neighbor = n
                break
        if smallest_neighbor is None:
            smallest_neighbor = smallest_neighbors[0]

        self.next_cell = smallest_neighbor
        self.explored.append(self.current_cell)

        # self.print_maze()

        return smallest_neighbor








    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
    
    def find_shortest_path(self, path):
        """
        This function finds the shortest path in a given path. It returns the shortest path and its length.
        Whenever a node is found more than once in the path, the function skips to the last time the node is found.
        """
        shortest_path = []
        i = 0
        while i < len(path):
            shortest_path.append(path[i])
            # check if the element is in the following elements
            if path[i] in path[i+1:]:
                print("Found: ", path[i])
                # skip to the last time the element is found
                i = path[i+1:].index(path[i]) + i + 2
            else:
                i += 1
        return shortest_path, len(shortest_path)


    def print_maze(self):
        """
        This function prints the maze with the flood array values. The maze is based on the current knowledge of the robot: the walls that it has detected.
        """

        msg = "Current knowledge of the maze:\n"

        for i in range(self.maze_matrix.shape[0]):
            msg += "\033[4m" + " "*4 if (i, 0) not in self.maze_matrix[i][0] else " "*4
        msg += "\033[0m" + "\n"
        
        for i in range(self.maze_matrix.shape[0]):
            for j in range(self.maze_matrix.shape[1]):
                neighbors = self.maze_matrix[i][j]
                # print underlined if there is bottom wall
                msg += "\033[4m" if (i + 1, j) not in neighbors else ""
                
                # print("\033[4m" if (i + 1, j) not in neighbors else "", end="")
                # print left wall
                msg += "│" if (i, j - 1) not in neighbors else " "
                # print("│" if (i, j - 1) not in neighbors else " ", end="")
                # print value
                msg += "\033[91m" if (i, j) == self.start else "\033[92m" if (i, j) == self.goal else "\033[94m" if (i, j) in self.explored else ""
                msg += "{:<2}".format(self.flood_matrix[i][j])
                msg += "\033[0m" if (i, j) == self.start or (i, j) == self.goal or (i, j) in self.explored else ""
                # print("\033[91m" if (i, j) == self.start else "\033[92m" if (i, j) == self.goal else "\033[94m" if (i, j) in self.explored else "", end="")
                # print("{:<2}".format(self.flood_matrix[i][j]), end="")
                # print("\033[0m" if (i, j) == self.start or (i, j) == self.goal or (i, j) in self.explored else "", end="")
                #print right wall
                msg += "│" if (i, j + 1) not in neighbors else " "
                msg += "\033[0m"
                # print("│" if (i, j + 1) not in neighbors else " ", end="")
                # print("\033[0m", end="")
            # print()
            msg += "\n"
    
        self.get_logger().info(msg)



def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.run()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
