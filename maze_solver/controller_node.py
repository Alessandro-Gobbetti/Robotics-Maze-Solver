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
from math import pi, sqrt, atan2, sin, cos

class Print:
    """
    This class simply the printing of colored text in the terminal.
    """
    RED = '\033[91m'
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'


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


        self.max_speed = 0.05
        self.max_angular_speed = 0.5

        self.odom_pose = None
        self.odom_velocity = None
        self.pose2d = None
        self.prev_cell_odom_pose = None
        self.start_pose = None
        self.current_pose = None
        # self.wall_state_robot = None
        self.wall_state_global = None
        self.distance_to_wall = None
        self.walls_threshold = 0.03
        self.is_rotation_needed = False

        self.floodfill_state = FloodFillState.REACH_GOAL
        self.explored = []
        self.best_paths = []
        self.best_path = None
        self.next_cell = None
        self.rotating = False
        self.going_to_next_cell = False
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        

        self.initial_rotation = pi
        self.dim = (8, 8)
        self.goal = (7, 0)
        self.start = (0, 0)
        self.flood_matrix = self.create_flood_matrix(self.dim, self.goal)
       
        # the maze array is a 2d array: each entry is a list of neighbors
        self.maze_matrix = self.create_maze_matrix(self.dim)
        
        
        self.READY = False
        self.is_stopped = False
        
        self.angular_threshold = 0.01 # Defines the threshold for the differ
        self.cell_side_length = 0.25 # Defines the length of the side of the cell
        # self.cell_area = self.cell_side_length ** 2
        # self.start_vel = 0.1 # Defines the start velocity of the cell
        # self.maze_side = 5.0 # Defines the length of the side of the maze
        # self.maze_area = 5.0**2 # The area of the maze that we are traversing
        # self.num_cells = self.maze_side // self.cell_side_length
        # self.maze_matrix = np.zeros((self.num_cells, self.num_cells)) # Creates the matrix by dividing the maze into cells that are defined as above. The elements denote how far we are from the goal.
        # self.start_cell = (0,0) # We start in the middle of the cell
        # self.dist_from_start = 0.0 # Defines how many cells away we are from the starting cell
        self.distance_tolerance = 0.02
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

        if self.next_cell and self.READY:
            self.goto(self.get_pose_from_cell(self.next_cell))

       

    def move_to_pose(self, goal_pose, current_pose):
        if self.euclidean_distance(goal_pose, current_pose) >= self.distance_tolerance:
            # We still haven't reached the goal pose. Use a proportional controller to compute velocities
            # that will move the turtle towards the goal (https://en.wikipedia.org/wiki/Proportional_control)
        
            # Twist represents 3D linear and angular velocities, in turtlesim we only care about 2 dimensions:
            # linear velocity along the x-axis (forward) and angular velocity along the z-axis (yaw angle)
            cmd_vel = Twist() 
            cmd_vel.linear.x = self.linear_vel(goal_pose, current_pose, constant=0.8)
            # TODO: adjust angular velocity based on the distance between the walls


            if self.proximities['left'] > 0 and self.proximities['right'] > 0:
                if self.distance_to_wall is None:
                    self.distance_to_wall = np.mean([self.proximities['left'], self.proximities['right']])
                    self.get_logger().info(f"Initializing distance to wall: {self.distance_to_wall}")

                # two walls, keep distance to both walls close to equal
                if abs(self.proximities['left'] - self.proximities['right']) > self.walls_threshold:
                    # turn right or left
                    cmd_vel.angular.z = (self.proximities['left'] - self.proximities['right'])
                    self.get_logger().info(f"Adjusting angular velocity: {cmd_vel.angular.z}")
                # oherwise, we are good!  
            elif self.proximities['left'] > 0:
                # keep distance to that wall
                if self.proximities['left'] > self.distance_to_wall + self.walls_threshold:
                    # turn right
                    cmd_vel.angular.z = self.distance_to_wall - self.proximities['left']
                    self.get_logger().info(f"Adjusting angular velocity LEFT >: {cmd_vel.angular.z}")
                elif self.proximities['left'] < self.distance_to_wall - self.walls_threshold:
                    # turn left
                    cmd_vel.angular.z = -(self.distance_to_wall - self.proximities['left'])
                    self.get_logger().info(f"Adjusting angular velocity LEFT <: {cmd_vel.angular.z}")
            elif self.proximities['right'] > 0:
                # keep distance to that wall
                if self.proximities['right'] < self.distance_to_wall - self.walls_threshold:
                    # turn right
                    cmd_vel.angular.z = self.distance_to_wall - self.proximities['right']
                    self.get_logger().info(f"Adjusting angular velocity RIGHT <: {cmd_vel.angular.z}")
                elif self.proximities['right'] > self.distance_to_wall + self.walls_threshold:
                    # turn left
                    cmd_vel.angular.z = -(self.distance_to_wall - self.proximities['right'])
                    self.get_logger().info(f"Adjusting angular velocity RIGHT >: {cmd_vel.angular.z}")
                # wait(10)
            else:
                # no walls, rely on odometry and hope
                cmd_vel.angular.z = self.angular_vel(goal_pose, current_pose)

            # cmd_vel.angular.z *= 3

            # If only one wall, keep distance to that wall fixed to precomputed value (the distance at the start?)

            # cmd_vel.angular.z = self.angular_vel(goal_pose, current_pose)
            # limit linear velocity in range [-max_speed, max_speed]
            cmd_vel.linear.x = max(min(cmd_vel.linear.x, self.max_speed), -self.max_speed)
            # limit angular velocity in range [-max_angular_speed, max_angular_speed]
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_speed), -self.max_angular_speed)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Goal reached, shutting down...")
            self.is_stopped = self.get_clock().now()
            self.stop()


    def rotate_in_place(self, goal_pose, current_pose):
        cmd_vel = Twist()
        cmd_vel.angular.z = self.angular_vel(goal_pose, current_pose)
        # limit angular velocity in range [-max_angular_speed, max_angular_speed]
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_speed), -self.max_angular_speed)
        cmd_vel.linear.x = 0.0
        self.vel_publisher.publish(cmd_vel)

        goal_theta = self.steering_angle(goal_pose, self.current_pose)

        if abs(self.angular_difference(goal_theta, self.current_pose[2])) < self.angular_threshold:
            self.get_logger().info("Done rotating in place")
            self.is_rotation_needed = False

    def goto(self, goal_pose):

        # if self.is_stopped:
        #     # wait for one second
        #     if self.get_clock().now().seconds_nanoseconds()[0] - self.is_stopped.seconds_nanoseconds()[0] < 1:
        #         self.stop()
        #         return
        #     else:
        #         self.is_stopped = False

        if self.is_rotation_needed:
            # Checks if there is a difference in the theta for the goal pose and the current pose (if we need to turn)
            self.rotate_in_place(goal_pose, self.current_pose)
        else:
            self.move_to_pose(goal_pose, self.current_pose) # We do not need to turn anymore and therefore we can go straight
        


    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose[1] - current_pose[1], goal_pose[0] - current_pose[0])

    def angular_vel(self, goal_pose, current_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)
        return constant * self.angular_difference(goal_theta, current_pose[2])

   

    def get_cell_from_pose(self, pose2d: tuple):
        """
        This function converts a given 2D pose in the odometry frame to cell coordinates in the maze matrix.
        It also computes the error in this conversion. If the error is 0.0, it means the robot is exactly at the center of the cell.
        The conversion depends on the initial rotation of the robot.

        Parameters
        ----------
        pose2d (tuple): A tuple (x, y) representing the 2D pose in the odometry frame.

        Returns
        -------
        cell (tuple): A tuple (x, y) representing the cell coordinates in the grid map.
        euclidean_error (float): The Euclidean distance between the actual pose and the center of the cell.
        """

        x = (pose2d[0] / self.cell_side_length) + self.start[0]
        y = (pose2d[1] / self.cell_side_length) + self.start[1]

        dir = round(self.initial_rotation / (pi/2)) # -2, -1, 0, 1, 2
        # if dir == -2 or dir == 2:
        #     x = x
        #     y = y
        # elif dir == -1:
        #     x = x
        #     y = y
        if dir == 0: # moving up, inverted x
            x = -x
            # y = y
        elif dir == 1: 
            # x = x
            y = -y

        # rotate the 2d coordinates
        # x = x * cos(orientation) - y * sin(orientation)
        # y = x * sin(orientation) + y * cos(orientation)

        # return the rounded coordinates + rounding error
        cell = (round(x), round(y))
        error = ((x - cell[0]) * self.cell_side_length, (y - cell[1]) * self.cell_side_length) 
        euclidean_error = sqrt(error[0]**2 + error[1]**2)
        return cell, euclidean_error

    def get_pose_from_cell(self, cell: tuple):
        """
        Converts a given cell coordinate in the maze matrix to a 2D pose in the odometry frame.

        Parameters
        ----------
        cell (tuple): A tuple (x, y) representing the cell coordinates in the maze matrix.

        Returns
        -------
        pose2d (tuple): A tuple (x, y) representing the 2D pose in the odometry frame.
        """
        x = cell[0] * self.cell_side_length
        y = cell[1] * self.cell_side_length
        dir = round(self.initial_rotation / (pi/2)) # -2, -1, 0, 1, 2
        if dir == 0:
            x = -x
        elif dir == 1:
            y = -y
        return (x, y)
        
    # --------------------------------------------------------------------------------------------
    # Callback functions

    def proximity_callback(self, message, which):
        self.READY = True
        self.proximities[which] = message.range # Updating the dict values with the current proximity value 
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        pose2d = self.pose3d_to_2d(self.odom_pose)
        # self.get_logger().info(f"Got Pose {pose2d}")
        self.current_pose = (pose2d[0], pose2d[1], pose2d[2])


        if not self.READY:
            return
        
        is_start = False
        if self.start_pose is None:
            is_start = True
            self.start_pose = (pose2d[0], pose2d[1], pose2d[2])

        current_cell, error = self.get_cell_from_pose(self.current_pose)
        if is_start or (error < self.distance_tolerance and current_cell != self.current_cell) or  0 < self.proximities['center'] < 0.05:
            self.get_logger().info(f"Current Cell: {current_cell}, Error: {error}, Pose: {self.current_pose}, Pose: {self.get_pose_from_cell(current_cell)}")
            self.current_cell = current_cell

            if self.floodfill_state == FloodFillState.REACH_GOAL and self.current_cell == self.goal :
                self.handle_goal_reached_exploring()

            if self.floodfill_state == FloodFillState.REACH_START and self.current_cell == self.start:
                self.handle_start_reached_exploiting()
                
            if self.floodfill_state == FloodFillState.SPRINT and self.current_cell == self.goal:
                self.handle_goal_reached_sprinting()

            # we are in the center of the cell
            if self.floodfill_state == FloodFillState.REACH_GOAL or self.floodfill_state == FloodFillState.REACH_START:
                self.detect_walls()
                self.next_cell = self.floodfill_step()
                
                # compute if we first need to rotate in place
                goal_pose = self.get_pose_from_cell(self.next_cell)
                goal_theta = self.steering_angle(goal_pose, self.current_pose)
                self.is_rotation_needed = abs(self.angular_difference(goal_theta, self.current_pose[2])) >= self.angular_threshold + pi/6

                self.get_logger().info(f"Next Cell: {self.next_cell}, is_rotation_needed: {self.is_rotation_needed}")
            elif self.floodfill_state == FloodFillState.SPRINT:
                if self.best_path:
                    self.next_cell = self.best_path.popLeft()
                    # compute if we first need to rotate in place
                    goal_pose = self.get_pose_from_cell(self.next_cell)
                    goal_theta = self.steering_angle(goal_pose, self.current_pose)
                    self.is_rotation_needed = abs(self.angular_difference(goal_theta, self.current_pose[2])) >= self.angular_threshold + pi/6

                    self.get_logger().info(f"Next Cell: {self.next_cell}")
                else:
                    self.stop()
                    self.get_logger().info("Done.")


    def handle_goal_reached_exploring(self):
        # compute the shortest path
        shortest_path, length = self.find_shortest_path(self.explored)
        self.best_paths.append(shortest_path)
        self.get_logger().info(f"Shortest Path: {shortest_path} with length {length}")

        # invert the flood matrix to reach the start
        self.flood_matrix = self.create_flood_matrix(self.dim, self.start)

        self.floodfill_state = FloodFillState.REACH_START
        self.get_logger().info("Goal reached. Returning to start.")

    def handle_start_reached_exploiting(self):
        # extract the path from goal to start form the explored list
        # find the goal node and split the list
        goal_index = self.explored.index(self.goal)
        explored = self.explored[:goal_index+1]
        self.get_logger().info(f"Explored: {self.explored}, Goal Index: {goal_index}, Explored: {explored})")
        explored.reverse()

        # compute the shortest path
        shortest_path, length = self.find_shortest_path(explored)
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

    def handle_goal_reached_sprinting(self):
        self.get_logger().info("Goal reached. Done.")

    
    # --------------------------------------------------------------------------------------------
    def check_walls(self):
        return WallState((int(self.proximities['left']>0), int(self.proximities['center']>0), int(self.proximities['right']>0), int(self.proximities['left_back']>0 and self.proximities['right_back']>0)))


    # wall detection and decision making
    def detect_walls(self):
        # walls_robot = WallState((int(0<self.proximities['left']<0.1), int(self.proximities['center']>0), int(0<self.proximities['right']<0.1), int(self.proximities['left_back']>0 and self.proximities['right_back']>0)))
        walls_robot = self.check_walls()
        self.get_logger().info(f"WALLS: {walls_robot}")
        self.get_logger().info(f"WALLS: {self.proximities}")
        walls = WallState(self.rotate(walls_robot.value, self.initial_rotation))
        self.wall_state_global = WallState(self.rotate(walls.value, self.current_pose[2]))
        
        old_neighbors = self.maze_matrix[self.current_cell[0]][self.current_cell[1]]
        new_neighbors = [n if not w else None for n, w in zip(self.maze_matrix[self.current_cell[0]][self.current_cell[1]], self.wall_state_global.value)]
        # remove current cell as neighbor for all the removed neighbors
        for i, n in enumerate(old_neighbors):
            if n is not None and new_neighbors[i] is None:
                self.maze_matrix[n[0]][n[1]] = [x if x != self.current_cell else None for x in self.maze_matrix[n[0]][n[1]]]

        self.maze_matrix[self.current_cell[0]][self.current_cell[1]] = new_neighbors
        
        self.print_maze()
        
        self.get_logger().info(f"\n -----------\n cell: {self.current_cell}\n  Walls : {walls_robot.value},\nRotated Walls: {walls.value}, \nGlobal Wall State: {self.wall_state_global.value} \n -----------")


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
        # print("neighbors: ", neighbors)
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
            max_iters = 100
            iters = 0

            while not len(queue) == 0:
                iters += 1
                if iters > max_iters:
                    exit()

                # TODO: detect the case in which the maze is not solvable
                el = queue.popleft()
                el_neighbors = self.maze_matrix[el[0]][el[1]]
                # print("el: ", el, el_neighbors)
                el_smallest_neighbor, el_smallest_neighbor_value = self.get_min_neighbors(el_neighbors, self.flood_matrix)
                if self.flood_matrix[el[0]][el[1]] <= el_smallest_neighbor_value:
                    # in case the current cell is smaller than all neighbors, floodfill the neighbors
                    self.flood_matrix[el[0]][el[1]] = el_smallest_neighbor_value + 1
                    for n in el_neighbors:
                        if n is not None and n not in queue:
                            queue.append(n)
                            # print("adding: ", n, "to queue: ", queue)
            
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
            msg += Print.UNDERLINE + " "*4 if (i, 0) not in self.maze_matrix[i][0] else " "*4
        msg += Print.END + "\n"
        
        for i in range(self.maze_matrix.shape[0]):
            for j in range(self.maze_matrix.shape[1]):
                neighbors = self.maze_matrix[i][j]
                # print underlined if there is bottom wall
                msg += Print.UNDERLINE if (i + 1, j) not in neighbors else ""
                msg += "│" if (i, j - 1) not in neighbors else " "
                msg += Print.RED if (i, j) == self.start else Print.GREEN if (i, j) == self.goal else Print.YELLOW if (i, j) == self.current_cell else Print.BLUE if (i, j) in self.explored else ""
                msg += "{:<2}".format(self.flood_matrix[i][j])
                msg += Print.END if (i, j) == self.start or (i, j) == self.goal or (i, j) in self.explored or (i, j) == self.current_cell else ""
                msg += "│" if (i, j + 1) not in neighbors else " "
                msg += Print.END 
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
