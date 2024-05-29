import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from collections import deque
from enum import Enum
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from math import pi, sqrt, atan2, atan, sin, cos
import matplotlib
import matplotlib.pyplot as plt


from .pid import PID
from .print import Print
from .enums import WallState, FloodFillState



class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_proximity')

        self.declare_parameter('maze', 'example_small')

        maze = self.get_parameter('maze').value
        if maze == 'example_small':
            self.dim = (4,4)
            self.start = (0,0)
            self.goal = (3,0)
        elif maze == 'example_big':
            self.dim = (8,8)
            self.start = (0,0)
            self.goal = (7,0)
        else:
            self.get_logger().error(f"Unknown maze: {maze}, please use 'example_small' or 'example_big' or provide your own maze")
            exit(1)


        self.max_speed = 0.04
        self.max_angular_speed = 0.4
        # self.pid = PID(3, 0, 0, -pi, pi)
        self.pid_left = PID(1, 0, 0, -pi, pi)
        self.pid_right = PID(1, 0, 0, -pi, pi)
        self.last_time = self.get_clock().now()
        self.dt = 0.0

        self.odom_error = (0,0,0)
        self.start_pose = None
        self.current_pose = None
        self.distance_to_wall = None
        self.walls_threshold = 0.01
        self.is_rotation_needed = False

        self.floodfill_state = FloodFillState.REACH_GOAL
        self.explored = []
        self.best_paths = []
        self.best_path = None
        self.next_cell = None
        self.error = None
        self.is_error_increasing = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/aseba/events/odometry', 10)

        self.initial_rotation = pi


        self.flood_matrix = self.create_flood_matrix(self.dim, self.goal)
        # the maze array is a 2d array: each entry is a list of neighbors
        self.maze_matrix = self.create_maze_matrix(self.dim)
        
        
        self.READY = False
        self.is_stopped = False
        
        self.angular_threshold = 0.03 # Defines the threshold for the differ
        self.cell_side_length = 0.25 # Defines the length of the side of the cell
        self.distance_tolerance = 0.02
        self.current_cell = (0,0)
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
        # def plot():
        #     if self.current_pose is not None:
        #         # plt.arrow(self.current_pose[0], self.current_pose[1], 0.05*cos(self.current_pose[2]), 0.05*sin(self.current_pose[2]), head_width=0.01, head_length=0.01, fc='r', ec='r')
        #         # plt.arrow(self.current_pose[0] - self.odom_error[0], self.current_pose[1] - self.odom_error[1], 0.05*cos(self.current_pose[2] - self.odom_error[2]), 0.05*sin(self.current_pose[2] - self.odom_error[2]), head_width=0.01, head_length=0.01, fc='b', ec='b')
        #         plt.plot(self.current_pose[0], self.current_pose[1], 'bo')
        #         plt.plot(self.current_pose[0] - self.odom_error[0], self.current_pose[1] - self.odom_error[1], 'ro')
        #         plt.savefig('plot.png')

        # self.timer2 = self.create_timer(1/5, plot)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    # --------------------------------------------------------------------------------------------
    # main loop

    def update_callback(self):

        if self.next_cell and self.READY:
            self.goto(self.get_pose_from_cell(self.next_cell))

       

    def move_forward(self):
        """
        Moves the robot forward while following the walls using proportional-integral-derivative (PID) control.

        The method calculates the linear and angular velocities of the robot based on the proximity sensor readings
        and the desired distance to the walls. It uses a PID controller to adjust the angular velocity for wall following.

        Returns:
            None

        """
        cmd_vel = Twist() 
        cmd_vel.linear.x = self.max_speed #self.linear_vel(goal_pose, current_pose, constant=0.8)
        
        l = self.proximities['left'] 
        lb = self.proximities['center_left']
        r = self.proximities['right']
        rb = self.proximities['center_right']
        c = self.proximities['center']
        dist = 0.066936

        if abs(c) < 0.1:
            cmd_vel.linear.x = self.max_speed/2 
        
        # update dt
        self.dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()


        def follow_left(l, lb, dist):
            phi = atan((l - lb)/ dist)
            d = 0.5 * (l + lb) - self.distance_to_wall
            alpha = phi + d
            return self.pid_left.step(alpha, self.dt)

        def follow_right(r, rb, dist):
            phi = atan((rb - r)/ dist)
            d = self.distance_to_wall - 0.5 * (r + rb)
            alpha = phi + d
            return self.pid_right.step(alpha, self.dt)

        if l > 0 and lb > 0 and (r < 0 or rb < 0):
            # using left wall only
            cmd_vel.angular.z = follow_left(l, lb, dist)
        if r > 0 and rb > 0 and (l < 0 or lb < 0):
            # using right wall only
            cmd_vel.angular.z = follow_right(r, rb, dist)
        if l > 0 and lb > 0 and r > 0 and rb > 0:
            # using both walls
            left_vel = follow_left(l, lb, dist)
            right_vel = follow_right(r, rb, dist)
            cmd_vel.angular.z = (right_vel + left_vel) / 2

        # cmd_vel.angular.z = self.angular_vel(goal_pose, current_pose)
        # limit linear velocity in range [-max_speed, max_speed]
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, self.max_speed), -self.max_speed)
        # limit angular velocity in range [-max_angular_speed, max_angular_speed]
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        # Publish the command
        self.vel_publisher.publish(cmd_vel)


    def rotate_in_place(self, goal_pose, current_pose):
        """
        Rotates the robot in place to align with the goal pose.

        Args:
            goal_pose (tuple): The desired pose to rotate towards, in the form (x, y, theta).
            current_pose (tuple): The current pose of the robot, in the form (x, y, theta).

        Returns:
            None

        Raises:
            None
        """

        if abs(self.angular_difference(self.steering_angle(goal_pose, self.current_pose), self.current_pose[2])) < self.angular_threshold:
            self.get_logger().info("Done rotating in place")
            self.stop()
            self.is_rotation_needed = False
        else:
            cmd_vel = Twist()
            cmd_vel.angular.z = self.angular_vel(goal_pose, current_pose)
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.max_angular_speed), -self.max_angular_speed)
            cmd_vel.linear.x = 0.0
            self.vel_publisher.publish(cmd_vel)

        
    def goto(self, goal_pose):
        """
        Moves the robot to the specified goal pose.

        Args:
            goal_pose: The desired pose to reach.

        Returns:
            None
        """
        if self.is_rotation_needed:
            # Checks if there is a difference in the theta for the goal pose and the current pose (if we need to turn)
            self.rotate_in_place(goal_pose, self.current_pose)
        else:
            self.move_forward() # We do not need to turn anymore and therefore we can go straight
        


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
        
        if dir == 0: # inverted x
            x = -x
        elif dir == 1: 
            y = -y

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
       
        odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        pose2d = self.pose3d_to_2d(odom_pose)

        if not self.READY:
            return 
        # ----------------------------------------------
        l = self.proximities['left']
        lb = self.proximities['center_left']
        r = self.proximities['right']
        rb = self.proximities['center_right']
        c = self.proximities['center']

        # calibrate the distance to the wall
        if self.distance_to_wall is None and l > 0 and r > 0 and lb > 0 and rb > 0:
            self.distance_to_wall = np.mean([l, r, lb, rb])
            self.get_logger().info(f"Initializing distance to wall: {self.distance_to_wall}, left: {l}, right: {r}, left_back: {lb}, right_back: {rb}")


        # use previous error for accuracy
        self.current_pose = pose2d

        # when moving forward, correct odometry based on the proximity sensors
        if not self.is_rotation_needed:
            if l > 0 and lb > 0 and r > 0 and rb > 0 and (abs(l-lb) < 0.01 or abs(r-rb) < 0.01):
                # we are in the center of the corridor
                # check orientation of the robot if vertical or horizontal
                pose = (pose2d[0] + self.odom_error[0], pose2d[1] + self.odom_error[1], pose2d[2])
                cell= self.get_cell_from_pose(pose)[0]
                coords = self.get_pose_from_cell(cell)

                dir = round(pose2d[2] / (pi/2)) # -2, -1, 0, 1, 2
                if dir in [-2, 0, 2]:
                    # we are moving parallel to the walls, so the angle should be a multiple of pi/2
                    corrected_angle = round(pose2d[2] / (pi/2)) * (pi/2) 
                    self.odom_error = (self.odom_error[0], coords[1] - pose2d[1], corrected_angle - pose2d[2])
                elif dir in [-1, 1]:
                    # we are moving parallel to the walls, so the angle should be a multiple of pi/2
                    corrected_angle = round(pose2d[2] / (pi/2)) * (pi/2) 
                    self.odom_error = (coords[0] - pose2d[0], self.odom_error[1], corrected_angle - pose2d[2])
                else:
                    self.get_logger().error(f"Error in orientation: {pose2d[2]}, {pose2d[2] % pi/2}, {abs(abs(pose2d[2]) % pi)}, {abs(abs(pose2d[2]) % pi/2)}")
            elif (l > 0 and lb > 0 and abs(l-lb) < 0.01) or (r > 0 and rb > 0 and abs(r-rb) < 0.01):
                corrected_angle = round(pose2d[2] / (pi/2)) * (pi/2)
                self.odom_error = (self.odom_error[0], self.odom_error[1], corrected_angle - pose2d[2])


            if c > 0:
                # correct odometry based on the center sensor
                pose = (pose2d[0] + self.odom_error[0], pose2d[1] + self.odom_error[1], self.current_pose[2])
                coords = self.get_pose_from_cell(self.get_cell_from_pose(pose)[0])
                if c < 0.035:
                    # when approaching the wall, we reset odometry to be at the center of the cell, the robot will stop
                    self.odom_error = (coords[0] - self.current_pose[0], coords[1] - self.current_pose[1], self.odom_error[2])
        # ----------------------------------------------

        # update new pose with improved error
        angle = (pose2d[2] + self.odom_error[2])
        if angle > pi:
            angle -= 2*pi
        elif angle < -pi:
            angle += 2*pi

        self.current_pose = (pose2d[0] + self.odom_error[0], pose2d[1] + self.odom_error[1], angle)
        
        
        
        is_start = False
        if self.start_pose is None:
            is_start = True
            self.start_pose = (pose2d[0], pose2d[1], pose2d[2])

        current_cell, error = self.get_cell_from_pose(self.current_pose)


        is_next_cell = False
        if self.next_cell is not None :
            distance_to_next_cell = self.euclidean_distance(self.get_pose_from_cell(self.next_cell), self.current_pose)
            is_next_cell = self.error is not None and self.is_error_increasing is not None and not self.is_error_increasing and distance_to_next_cell >= self.error and current_cell != self.current_cell and not self.is_rotation_needed

            if self.error is not None:
                self.is_error_increasing = distance_to_next_cell > self.error
            self.error = distance_to_next_cell  


        if is_start or is_next_cell:
            self.error = None
            self.is_error_increasing = None
            self.get_logger().info(f"Current Cell: {current_cell}, Error: {error}, Pose: {self.current_pose}, Pose: {self.get_pose_from_cell(current_cell)}")
            self.current_cell = current_cell

            if self.floodfill_state == FloodFillState.REACH_GOAL and self.current_cell == self.goal :
                self.handle_goal_reached_exploring()

            if self.floodfill_state == FloodFillState.REACH_START and self.current_cell == self.start:
                self.handle_start_reached_exploring()
                
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
                if self.is_rotation_needed:
                    self.stop()

                self.get_logger().info(f"Next Cell: {self.next_cell}, is_rotation_needed: {self.is_rotation_needed}")
            elif self.floodfill_state == FloodFillState.SPRINT:
                if len(self.best_path) > 0:
                    self.print_maze()
                    self.next_cell = self.best_path.pop(0)
                    # compute if we first need to rotate in place
                    goal_pose = self.get_pose_from_cell(self.next_cell)
                    goal_theta = self.steering_angle(goal_pose, self.current_pose)
                    self.is_rotation_needed = abs(self.angular_difference(goal_theta, self.current_pose[2])) >= self.angular_threshold + pi/6
                    if self.is_rotation_needed:
                        self.stop()

                    self.get_logger().info(f"Next Cell: {self.next_cell}")
                else:
                    self.stop()
                    self.handle_goal_reached_sprinting()


    def handle_goal_reached_exploring(self):
        """
        Handles the event when the goal is reached during the exploring phase.

        This method computes the shortest path from the current cell to the goal,
        updates the best_paths list with the shortest path, and logs the information
        about the shortest path and its length. It then sets the floodfill_state to
        REACH_START and updates the flood_matrix to reach the start.

        Returns:
            None
        """
        # compute the shortest path
        explored = self.explored
        explored.append(self.current_cell)
        shortest_path, length = self.find_shortest_path(explored)
        self.best_paths.append(shortest_path)
        self.get_logger().info(f"Shortest Path: {shortest_path} with length {length}")

        # invert the flood matrix to reach the start
        self.flood_matrix = self.create_flood_matrix(self.dim, self.start)

        self.floodfill_state = FloodFillState.REACH_START
        self.get_logger().info("Goal reached. Returning to start.")

    def handle_start_reached_exploring(self):
        """
        Handles the event when the start cell is reached during exploration.

        This method extracts the path from the goal to the start from the explored list,
        computes the shortest path, and updates the best path based on the length of the paths found.

        Returns:
            None
        """
        # extract the path from goal to start form the explored list
        # find the goal node and split the list
        explored = self.explored + [self.current_cell]
        goal_index = explored.index(self.goal)
        explored = explored[goal_index:]
        self.get_logger().info(f"Explored: {self.explored}, Goal Index: {goal_index}, Explored now: {explored})")
        # reverse the list
        explored = explored[::-1]

        # compute the shortest path
        shortest_path, length = self.find_shortest_path(explored)
        self.best_paths.append(shortest_path)
        self.get_logger().info(f"Shortest Path: {shortest_path} with length {length}")

        if len(self.best_paths) > 1:
            self.get_logger().info("Multiple paths found. Deciding the best path...")
            for i, path in enumerate(self.best_paths):
                self.get_logger().info(f"Path {i}: {path}, Length: {len(path)}")
            self.best_path = min(self.best_paths, key=lambda x: len(x))
        else:
            self.best_path = self.best_paths[0]

        if self.best_path[0] == self.start:
            self.best_path.pop(0)

        self.floodfill_state = FloodFillState.SPRINT
        self.get_logger().info("Sprint Mode Activated.")
        self.get_logger().info(f"Best Path: {self.best_path}, Length: {len(self.best_path)}")

    def handle_goal_reached_sprinting(self):
        """
        Handles the action when the goal is reached in sprinting mode.
        Stops the robot and exits the program.
        """
        self.get_logger().info("Goal reached. Done!")
        self.stop()
        exit()

    
    # --------------------------------------------------------------------------------------------
    def check_walls(self):
        """
        Check the proximity sensors for walls and return the wall state.

        Returns:
            WallState: A tuple representing the presence of walls in different directions.
                        The tuple contains four elements: (left, center, right, back).
                        Each element is a boolean value, where True indicates the presence of a wall
                        and False indicates no wall.
        """
        return WallState((int(self.proximities['left']>0), int(self.proximities['center']>0), int(self.proximities['right']>0), 0))


    # wall detection and decision making
    def detect_walls(self):
        """
        Detects the walls surrounding the current cell and updates the maze matrix accordingly.

        This method checks the proximity sensor readings to determine the presence of walls in the left, center, right, left back, and right back directions.
        It then rotates the wall state based on the initial rotation and current pose of the robot.
        The old neighbors of the current cell are updated based on the removed walls.
        Finally, the maze matrix is updated with the new wall state of the current cell.

        Returns:
            None
        """
        walls_robot = self.check_walls()
        walls = WallState(self.rotate(walls_robot.value, self.initial_rotation))
        wall_state_global = WallState(self.rotate(walls.value, self.current_pose[2]))
        
        old_neighbors = self.maze_matrix[self.current_cell[0]][self.current_cell[1]]
        new_neighbors = [n if not w else None for n, w in zip(self.maze_matrix[self.current_cell[0]][self.current_cell[1]], wall_state_global.value)]
        # remove current cell as neighbor for all the removed neighbors
        for i, n in enumerate(old_neighbors):
            if n is not None and new_neighbors[i] is None:
                self.maze_matrix[n[0]][n[1]] = [x if x != self.current_cell else None for x in self.maze_matrix[n[0]][n[1]]]

        self.maze_matrix[self.current_cell[0]][self.current_cell[1]] = new_neighbors
        
        self.print_maze()
        
        self.get_logger().info(f"\n -----------\n cell: {self.current_cell}\nWalls robot perspective: {walls_robot.value},\nGlobal Wall State: {wall_state_global.value} \n -----------")


    def rotate(self, vec, angle_pi):
        """
        Rotates the given vector by the specified angle.

        Args:
            vec (list): The vector to be rotated.
            angle_pi (float): The angle of rotation in multiples of pi.

        Returns:
            list: The rotated vector.
        """
        angle_pi = round(angle_pi/(pi/2)) % 4
        return vec[angle_pi:] + vec[:angle_pi]
    
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



    # --------------------------------------------------------------------------------------------
    # Flood Fill functions
    def create_flood_matrix(self, dim, goal):
        """
        Creates a flood matrix for a given maze dimension and goal position.

        Args:
            dim (tuple): The dimension of the maze represented as a tuple (rows, columns).
            goal (tuple): The goal position represented as a tuple (row, column).

        Returns:
            numpy.ndarray: The flood matrix representing the distance of each cell to the goal.
        """
        flood_mat = np.zeros(dim, dtype=int)
        # each cell is filled with the distance to the goal
        for i in range(dim[0]):
            for j in range(dim[1]):
                flood_mat[i][j] = abs(i - goal[0]) + abs(j - goal[1])
        return flood_mat
    
    def create_maze_matrix(self, dim):
        """
        Creates a maze matrix with given dimensions.

        Args:
            dim (tuple): A tuple representing the dimensions of the maze matrix.

        Returns:
            numpy.ndarray: The maze matrix with dimensions `dim`.

        """
        maze_mat = np.zeros(dim, dtype=object)
        for i in range(dim[0]):
            for j in range(dim[1]):
                neighbors = [(i + dx, j + dy) 
                             if (0 <= i + dx < dim[0] and 0 <= j + dy < dim[1]) else None                             
                             for dx, dy in [(0,-1), (-1,0), (0, 1), (1, 0)]
                             ]
                maze_mat[i][j] = neighbors
        return maze_mat

    def get_min_neighbors(self, neighbors, flood_array):
        """
        Returns the minimum value neighbors and its corresponding flood value from the given neighbors list and flood array.

        Args:
            neighbors (list): List of neighbor coordinates.
            flood_array (list): 2D array representing the flood values of the maze.

        Returns:
            tuple: A tuple containing two elements:
                - A list of neighbor coordinates with the minimum flood value.
                - The minimum flood value among the neighbors.
        """
        min_value = min([flood_array[n[0]][n[1]] for n in neighbors if n])
        return [n for n in neighbors if n and flood_array[n[0]][n[1]] == min_value], min_value

    


    def floodfill_step(self):
        """
        Performs a single step of the floodfill algorithm to explore the maze.

        Returns:
            tuple: The coordinates of the next cell to explore.
        """
        neighbors = self.maze_matrix[self.current_cell[0]][self.current_cell[1]]
        queue = deque()

        # get all the neighbors with the smallest value
        smallest_neighbors, smallest_neighbor_value = self.get_min_neighbors(neighbors, self.flood_matrix)

        # if current smaller than all neighbors, floodfill the neighbors
        if self.flood_matrix[self.current_cell[0]][self.current_cell[1]] < smallest_neighbor_value:
            queue.append(self.current_cell)
            max_iters = 1000
            iters = 0

            while not len(queue) == 0:
                iters += 1
                if iters > max_iters:
                    self.get_logger().error("The Maze is not solvable.")
                    self.stop()
                    exit(1)
                el = queue.popleft()
                el_neighbors = self.maze_matrix[el[0]][el[1]]
                el_smallest_neighbor, el_smallest_neighbor_value = self.get_min_neighbors(el_neighbors, self.flood_matrix)
                if self.flood_matrix[el[0]][el[1]] <= el_smallest_neighbor_value:
                    # in case the current cell is smaller than all neighbors, floodfill the neighbors
                    self.flood_matrix[el[0]][el[1]] = el_smallest_neighbor_value + 1
                    for n in el_neighbors:
                        if n is not None and n not in queue:
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

        return smallest_neighbor


    
    
    def find_shortest_path(self, path):
        """
        This function finds the shortest path in a given path. It returns the shortest path and its length.
        Whenever a node is found more than once in the path, the function skips to the last time the node is found.
        
        Args:
            path (list): A list representing the path to be analyzed.
            
        Returns:
            tuple: A tuple containing the shortest path (list) and its length (int).
        """
        shortest_path = []
        i = 0
        while i < len(path):
            shortest_path.append(path[i])
            # check if the element is in the following elements
            if path[i] in path[i+1:]:
                # skip to the last time the element is found
                i = path[i+1:].index(path[i]) + i + 2
            else:
                i += 1
        return shortest_path, len(shortest_path)


    def print_maze(self):
        """
        This function prints the maze with the flood array values.
        The maze is based on the current knowledge of the robot: the walls that it has detected.

        Returns:
            None
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
