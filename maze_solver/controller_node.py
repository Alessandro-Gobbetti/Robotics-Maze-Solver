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


class WallState(Enum):
    #(WEST, NORTH, EAST, SOUTH)
    #zero wall
    ZERO = (0,0,0,0)
    #one wall
    LEFT = (1,0,0,0)
    FRONT = (0,1,0,0)
    RIGHT = (0,0,1,0)
    BACK = (0,0,0,1)
    #two walls
    FRONT_LEFT = (1,1,0,0)
    RIGHT_LEFT = (1,0,1,0)
    BACK_LEFT = (1,0,0,1)
    FRONT_RIGHT = (0,1,1,0)
    BACK_RIGHT = (0,0,1,1)
    FRONT_BACK = (0,1,0,1)
    #three walls
    FRONT_LEFT_RIGHT = (1,1,1,0)
    RIGHT_LEFT_BACK = (1,0,1,1)
    BACK_LEFT_FRONT = (1,1,0,1)   
    FRONT_RIGHT_BACK = (0,1,1,1)
    #four walls
    FRONT_LEFT_RIGHT_BACK = (1,1,1,1)   

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
        self.queue = deque()
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        

        self.dim = (5,5)
        self.goal = (3,4)
        self.start = (4, 0)
        self.flood_matrix = self.create_fload_array(self.dim, self.goal)

        # the maze array is a 2d array: each entry is a list of neighbors
        self.maze_matrix = self.create_maze_matrix(self.dim)
        


        

        self.cell_side_length = 0.1 # Defines the length of the side of the cell
        # self.cell_area = self.cell_side_length ** 2
        # self.start_vel = 0.1 # Defines the start velocity of the cell
        self.maze_side = 5.0 # Defines the length of the side of the maze
        # self.maze_area = 5.0**2 # The area of the maze that we are traversing
        self.num_cells = self.maze_side // self.cell_side_length
        self.maze_matrix = np.zeros((self.num_cells, self.num_cells)) # Creates the matrix by dividing the maze into cells that are defined as above. The elements denote how far we are from the goal.
        self.start_cell = (0,0) # We start in the middle of the cell
        self.dist_from_start = 0.0 # Defines how many cells away we are from the starting cell
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
        current_odom_pose = self.new_pose
        current_cell_coord = self.current_cell
        target_x = cell_coord[0] - current_cell_coord[0]
        target_y = cell_coord[1] = current_cell_coord[1]

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

            # Detect walls
            self.detect_walls()
            self.next_cell = self.floodfill_step()


            self.prev_cell_odom_pose = (self.new_pose[0], self.new_pose[1], self.new_pose[2])
            
        #self.get_logger().info(
        #    "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        #)
    
    # --------------------------------------------------------------------------------------------
    # wall detection and decision making
    def detect_walls(self):

        walls = WallState((int(self.proximities['left']>0),int(self.proximities['center']>0),int(self.proximities['right']>0),int(self.proximities['rear_left']>0 and self.proximities['rear_back']>0)))
        self.wall_state_global = self.rotate(walls, self.new_pose[2])
        
        # update the maze matrix with the new wall state
        
        

        print(self.wall_state_global)

    # vec = [WALL_LEFT, WALL_FRONT, WALL_RIGHT, WALL_BACK]
    # rotate 90 degrees
    def rotate(vec, angle_pi):
        angle_pi = int(angle_pi/(pi/2)) % 4
        return vec[angle_pi:] + vec[:angle_pi]
        


    # TODO: floodfill algorithm

    # --------------------------------------------------------------------------------------------
    # Helper functions
    def create_fload_array(dim, goal):
        flood_array = np.zeros(dim, dtype=int)
        # each cell is filled with the distance to the goal
        for i in range(dim[0]):
            for j in range(dim[1]):
                flood_array[i][j] = abs(i - goal[0]) + abs(j - goal[1])
        return flood_array
    
    def create_maze_matrix(dim):
        maze_mat= np.zeros(dim, dtype=object)
        for i in range(dim[0]):
            for j in range(dim[1]):
                neighbors = [(i + dx, j + dy) 
                             if (0 <= i + dx < dim[0] and 0 <= j + dy < dim[1]) else None                             
                             for dx, dy in [(0,-1), (-1,0), (0, 1), (1, 0)]
                             ]
                maze_mat[i][j] = neighbors
        return maze_mat
    
    


    # def update_callback(self):     
    #     if self.flood_fill_state == FloodFillState.REACH_GOAL:
        
    #         self.update_detect()
    #     elif self.mode == ThymioState.DECIDE:
    #         self.update_decide()
    #     elif self.mode == ThymioState.WALK:
    #     # Let's just set some hard-coded velocities in this example
    #         cmd_vel = Twist() 
    #         cmd_vel.linear.x  = 0.2 # [m/s]
    #         cmd_vel.angular.z = 0.0 # [rad/s]
    #         # Publish the command
    #         self.vel_publisher.publish(cmd_vel)
    #     elif self.mode == ThymioState.DONE:
    #         return 
    

    def floodfill_step(self):
        neighbors = self.maze_array[self.current_cell[0]][self.current_cell[1]]
        
        





    def flood_fill(maze_array, flood_array, start, goal, walls_fn, explored=[]):
        path = [start]
        current = start

        queue = deque()

        while current != goal:
            # find the value of the neighboring cell with the smallest value
            neighbors = maze_array[current[0]][current[1]]
            neighbors = walls_fn(current, neighbors) # detect neighbors considering walls
            update_maze(maze_array, current, neighbors) # update the knowledge of the maze

            # return all the neighbors with the smallest value
            smallest_neighbors, smallest_neighbor_value = get_min_neighbors(neighbors, flood_array)

            # if current smaller than smallest neighbor
            if flood_array[current[0]][current[1]] < smallest_neighbor_value:
                queue.append(current)

                while not len(queue) == 0:
                    el = queue.popleft()
                    el_neighbors = maze_array[el[0]][el[1]]
                    el_smallest_neighbor = min(el_neighbors, key=lambda x: flood_array[x[0]][x[1]])
                    el_smallest_neighbor_value = flood_array[el_smallest_neighbor[0]][el_smallest_neighbor[1]]
                    # if el smaller than smallest neighbor
                    if flood_array[el[0]][el[1]] <= el_smallest_neighbor_value:
                        # print("Updating: ", el)
                        flood_array[el[0]][el[1]] = el_smallest_neighbor_value + 1
                        for n in el_neighbors:
                            queue.append(n)
                
                # recompute the smallest neighbors
                smallest_neighbors, smallest_neighbor_value = get_min_neighbors(neighbors, flood_array)

            # pick one of the neighbors, prefer the one that is not in the explored list
            smallest_neighbor = None
            for n in smallest_neighbors:
                if n not in explored:
                    smallest_neighbor = n
                    break
            if smallest_neighbor is None:
                smallest_neighbor = smallest_neighbors[0]
                            
            current = smallest_neighbor
            path.append(current)
            print_maze_walls(flood_array, start, goal, current, walls_fn)
            print()

        print_maze_path(flood_array, start, goal, path, walls_fn)
        print(path)
        
        return maze_array, flood_array, path
    












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



    


    

    def update_decide(self):
        pass

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
