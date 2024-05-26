import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Image
from math import pi, sqrt, atan2, sin, cos
import cv2
from cv_bridge import CvBridge
import tf_transformations
from nav_msgs.msg import Odometry
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

class ThymioState(Enum):
    DETECT = 1
    DECIDE = 2
    WALK = 3
    DONE = 4

class Position(Enum):
    UP = 1
    RIGHT = 2
    LEFT = 3
    DOWN = 4

class CallbackUsage(Enum):
    CAMERA = 1
    ROT = 2
    CALC = 3
    MOVE_A_LITTLE_FORWARD = 4

class RotationVal(Enum):
    NINETY_LEFT = 1
    NINETY_RIGHT = 2
    BACK = 3

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


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.current_pose = None
        self.wall_state_global = None
        self.is_rotation_needed = False
        self.floodfill_state = FloodFillState.REACH_GOAL
        self.explored = []
        self.best_paths = []
        self.best_path = None
        self.next_cell = None
        self.current_pose = None
        self.odom_pose = None

        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            
        # Creating an image subscriber:
        self.camera = self.create_subscription(Image, 'camera', self.camera_callback, 10)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/image_processed', 10)

        self.initial_rotation = pi
        self.dim = (8, 8)
        self.goal = (7, 0)
        self.start = (0, 0)
        self.flood_matrix = self.create_flood_matrix(self.dim, self.goal)
        self.current_cell = self.start
        # the maze array is a 2d array: each entry is a list of neighbors
        self.maze_matrix = self.create_maze_matrix(self.dim)
        
        
        self.READY = False
        
        self.angular_threshold = 0.01 # Defines the threshold for the differ
        self.orientation = Position.DOWN

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
        self.current_callback_dependency = CallbackUsage.CALC
        self.aligned = False
        self.rotation_start_time = None
        # self.expected_rot_time_end = None
        self.cur_rot = None # Stores the enum value of how much we need to rotate by.
        self.rot_vel = 0.2 # This is the angular velocity which will help the robot to rotate around its axis
        self.prev_cell_contour = None   # Stores contour center
        self.start_pose = None
        self.distance_threshold = 0.05
        self.distance_travel_by_itself = 0.08
        self.target_orientation = None
        # self.use_odometry = False
        self.angular_threshold = 0.01
        self.area_threshold = 1000

    def run(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.move)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)


    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        pose2d = self.pose3d_to_2d(self.odom_pose)
        # self.get_logger().info(f"Got Pose {pose2d}")
        self.current_pose = (pose2d[0], pose2d[1], pose2d[2])



    def camera_callback(self, message):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')

        # Convert the image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        # Define the red color range
        lower_red = np.array([0, 70, 50])
        upper_red = np.array([10, 255, 255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Find contours of the red square
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            #self.get_logger().info(f"Area: {area}")

            # Get the bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate the center of the red square
            square_center_x = x + w / 2
            square_center_y = y + h / 2

            # Calculate the offset from the image center
            image_center_x = cv_image.shape[1] / 2
            image_center_y = cv_image.shape[0] / 2

            if self.current_callback_dependency == CallbackUsage.CAMERA and area > self.area_threshold:

                offset_x = square_center_x - image_center_x
                # offset_y = square_center_y - image_center_y
                #self.get_logger().info(f"Current Offsets x:{offset_x}, y:{offset_y}")

                # Command to align the robot with the red square
                cmd_vel = Twist()
                    # Adjust angular velocity to align with the red square
                if abs(offset_x) > 0.5:  # Threshold to avoid small jitters
                    #self.get_logger().info(f"Adjusting Angle, current offset x:{offset_x}, square_c_x: {square_center_x}, image_c_x: {image_center_x}")
                    self.aligned = False
                    cmd_vel.angular.z = -0.03 * offset_x  # Adjust turning speed
                    # self.prev_cell_contour = None
                    # self.vel_publisher.publish(cmd_vel)
                else:
                    # self.aligned = True  # Aligned when x offset is within the threshold
                    cmd_vel.angular.z = 0.0

                # if self.aligned:
                cmd_vel.linear.x = 0.03
                # self.get_logger().info("Moving to the centre of the next cell")
                if self.prev_cell_contour is None:
                    # self.get_logger().info(f"Setting pcc: x: {square_center_x}, y: {square_center_y}")
                    self.prev_cell_contour = (square_center_x, square_center_y)
                # self.get_logger().info(f"cx: {square_center_x}, cy: {square_center_y}, pcx: {self.prev_cell_contour[0]}, pcy: {self.prev_cell_contour[1]}")
                movement_threshold = 50
                if (abs(square_center_x - self.prev_cell_contour[0]) > movement_threshold or
                    abs(square_center_y - self.prev_cell_contour[1]) > movement_threshold):
                    # self.get_logger().info(f"Got Sudden cell change, stopping and updating")
                    # self.get_logger().info(f"Current contour and prev contour centers:")
                    # self.get_logger().info(f"cx: {square_center_x}, cy: {square_center_y}, pcx: {self.prev_cell_contour[0]}, pcy: {self.prev_cell_contour[1]}")
                    self.stop()
                    # self.update_position()
                    self.prev_cell_contour = None
                    self.current_callback_dependency = CallbackUsage.MOVE_A_LITTLE_FORWARD
                    # self.current_callback_dependency = CallbackUsage.CALC
                else:
                    self.vel_publisher.publish(cmd_vel)
                    self.prev_cell_contour = (square_center_x, square_center_y)
            

            #Draw the bounding box and center on the image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (int(square_center_x), int(square_center_y)), 5, (255, 0, 0), -1)
            cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), 5, (0, 255, 255), -1)
        else:
            if self.current_callback_dependency == CallbackUsage.CAMERA:
                # self.get_logger().info(f"No contours present. Wall blocking front view.")
                self.stop()

                self.prev_cell_contour = None
                self.current_callback_dependency = CallbackUsage.MOVE_A_LITTLE_FORWARD

        # Convert the processed image and mask back to ROS Image messages
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')

        # Publish the processed image
        self.publisher_.publish(processed_image_msg)

    # --------------------------------------------------------------------------------------------
    # main loop
        
    def move(self):
        if self.current_pose is not None:
            match(self.current_callback_dependency):
                case CallbackUsage.CAMERA:
                    pass
                case CallbackUsage.CALC:
                    if self.floodfill_state == FloodFillState.REACH_GOAL and self.current_cell == self.goal:
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
                        need_rotation = self.check_rotation_need()
                        if need_rotation:
                            self.get_logger().info(f"In need of rotation")
                            self.current_callback_dependency = CallbackUsage.ROT    # We go to the state where we rotate the robot
                        else:   
                            self.get_logger().info(f"No need of rotation. Handing to camera")
                            self.current_callback_dependency = CallbackUsage.CAMERA # Else we just move the robot forward

                        self.get_logger().info(f"Next Cell: {self.next_cell}, is_rotation_needed: {need_rotation}")
                    elif self.floodfill_state == FloodFillState.SPRINT:
                        if self.best_path:
                            self.next_cell = self.best_path.pop(0)
                            # compute if we first need to rotate in place
                            need_rotation = self.check_rotation_need()
                            if need_rotation:
                                self.get_logger().info(f"In need of rotation")
                                self.current_callback_dependency = CallbackUsage.ROT    # We go to the state where we rotate the robot
                            else:   
                                self.get_logger().info(f"No need of rotation. Handing to camera")
                                self.current_callback_dependency = CallbackUsage.CAMERA # Else we just move the robot forward

                            self.get_logger().info(f"Next Cell: {self.next_cell}, is_rotation_needed: {need_rotation}")
                        else:
                            self.stop()
                            self.get_logger().info("Done.")

                case CallbackUsage.ROT:
                    if self.cur_rot is None or self.orientation is None:
                        self.get_logger().error(f"No rotation or orientation specified. Quitting..")
                        self.stop()
                        exit(1)
                    match(self.cur_rot):
                        case RotationVal.NINETY_LEFT:
                            self.turn_ninety_left()

                        case RotationVal.NINETY_RIGHT:
                            self.turn_ninety_right()

                        case RotationVal.BACK:
                            self.turn_back()

                        case _:
                            self.get_logger().error(f"Invalid rotation value specified: {self.cur_rot}. Aborting")
                            self.stop()
                            exit(1)

                case CallbackUsage.MOVE_A_LITTLE_FORWARD:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.1
                    # self.get_logger().info("Moving a bit forward by itself")
                    if self.start_pose is None: 
                        self.start_pose = self.current_pose
                    if self.euclidean_distance(self.start_pose, self.current_pose) > self.distance_travel_by_itself or (self.proximities['center'] != -1.0 and self.proximities['center'] < self.distance_threshold):
                        # self.get_logger().info("Travelled or encountered a wall in the front")
                        cmd_vel.linear.x = 0.0
                        self.update_position()
                        # self.get_logger().info("Position Updated")
                        self.current_callback_dependency = CallbackUsage.CALC
                        self.start_pose = None  # Setting it none for the next iteration.
                    self.vel_publisher.publish(cmd_vel)

                case _:
                    self.get_logger().error(f"No case matched. Please start execution again...")
                    self.stop()
                    exit(1)
        else:
            self.get_logger().info("Waiting for odometry to setup")


    def turn_ninety_right(self):
        # self.get_logger().info(f"Turning RIGHT")
        cp = self.pose3d_to_2d(self.odom_pose)
        ta = np.pi/2 + 0.2
        ca = cp[2]
        cmd_vel = Twist()
        if self.target_orientation is None:
            self.target_orientation = (ca - ta)%(2*np.pi)
        # self.get_logger().info(f"The target {self.target_orientation}, ca = {ca}")
        # self.get_logger().info(f"ad: {self.angular_difference(self.target_orientation, ca)}")
        # if np.abs(self.target_orientation - ca) < self.angular_threshold:
        if np.abs(self.angular_difference(self.target_orientation, ca)) < self.angular_threshold:
        
            self.stop()
            self.get_logger().info(f"Successfully Rotated 90 right. Stopping..")
            self.cur_rot = None
            self.target_orientation = None
            self.current_callback_dependency = CallbackUsage.CAMERA
        else:
            cmd_vel.angular.z = -0.3
            self.vel_publisher.publish(cmd_vel)
        
        
    def turn_ninety_left(self):
        # self.get_logger().info(f"Turning LEFT")
        cp = self.pose3d_to_2d(self.odom_pose)
        ta = np.pi/2 + 0.2
        ca = cp[2]
        cmd_vel = Twist()
        if self.target_orientation is None:
            self.target_orientation = (ca + ta)%(2*np.pi)
        # self.get_logger().info(f"The target {self.target_orientation}, ca = {ca}")
        # self.get_logger().info(f"ad: {self.angular_difference(self.target_orientation, ca)}")
        # if np.abs(self.target_orientation - ca) < self.angular_threshold:
        if np.abs(self.angular_difference(self.target_orientation, ca)) < self.angular_threshold:
        
            self.stop()
            self.get_logger().info(f"Successfully Rotated 90 Left. Stopping..")
            self.cur_rot = None
            self.target_orientation = None
            self.current_callback_dependency = CallbackUsage.CAMERA
        else:
            cmd_vel.angular.z = 0.3
            self.vel_publisher.publish(cmd_vel)

    def turn_back(self):
        # self.get_logger().info(f"Turning BACK")
        cp = self.pose3d_to_2d(self.odom_pose)
        ta = np.pi + 0.4
        ca = cp[2]
        cmd_vel = Twist()
        if self.target_orientation is None:
            # self.target_orientation = (ca + ta)%(2*np.pi)
            self.target_orientation = ca + ta
            # if self.target_orientation > np.pi:
            #     self.target_orientation -= 2*np.pi
        # self.get_logger().info(f"The target {self.target_orientation}, ca = {ca}")
        self.get_logger().info(f"ad: {self.angular_difference(self.target_orientation, ca)}")
        # if np.abs(self.target_orientation - ca) < self.angular_threshold:
        if np.abs(self.angular_difference(self.target_orientation, ca)) < self.angular_threshold:
            self.stop()
            self.get_logger().info(f"Successfully Rotated 180 Back. Stopping..")
            self.cur_rot = None
            self.target_orientation = None
            self.current_callback_dependency = CallbackUsage.CAMERA
        else:
            cmd_vel.angular.z = 0.3
            self.vel_publisher.publish(cmd_vel)


    def update_position(self):
        if self.orientation == Position.DOWN:
            self.current_cell = (self.current_cell[0] + 1, self.current_cell[1])
        elif self.orientation == Position.RIGHT:
            self.current_cell = (self.current_cell[0], self.current_cell[1] + 1)
        elif self.orientation == Position.UP:
            self.current_cell = (self.current_cell[0] - 1, self.current_cell[1])
        elif self.orientation == Position.LEFT:
            self.current_cell = (self.current_cell[0], self.current_cell[1] - 1)
        self.get_logger().info(f"Moved to cell: {self.current_cell}")

    def check_rotation_need(self):
        x1, y1 = self.current_cell
        x2, y2 = self.next_cell

        match(self.orientation):
            case Position.DOWN:
                if x1 < x2 and y1 == y2:
                    return False
                elif x1 > x2 and y1 == y2:
                    self.cur_rot = RotationVal.BACK
                    self.orientation = Position.UP
                    return True
                elif x1 == x2 and y1 < y2:
                    self.cur_rot = RotationVal.NINETY_LEFT
                    self.orientation = Position.RIGHT
                    return True
                elif x1 == x2 and y1 > y2:
                    self.cur_rot = RotationVal.NINETY_RIGHT
                    self.orientation = Position.LEFT
                    return True
                else:
                    self.get_logger().error(f"Not a valid next cell value. Restart...")
                    self.stop()
                    exit(1)
            case Position.UP:
                if x1 < x2 and y1 == y2:
                    self.cur_rot = RotationVal.BACK
                    self.orientation = Position.DOWN
                    return True
                elif x1 > x2 and y1 == y2:
                    return False
                elif x1 == x2 and y1 < y2:
                    self.cur_rot = RotationVal.NINETY_RIGHT
                    self.orientation = Position.RIGHT
                    return True
                elif x1 == x2 and y1 > y2:
                    self.cur_rot = RotationVal.NINETY_LEFT
                    self.orientation = Position.LEFT
                    return True
                else:
                    self.get_logger().error(f"Not a valid next cell value. Restart...")
                    self.stop()
                    exit(1)
            case Position.LEFT:
                if x1 < x2 and y1 == y2:
                    self.cur_rot = RotationVal.NINETY_LEFT
                    self.orientation = Position.DOWN
                    return True
                elif x1 > x2 and y1 == y2:
                    self.cur_rot = RotationVal.NINETY_RIGHT
                    self.orientation = Position.UP
                    return True
                elif x1 == x2 and y1 < y2:
                    self.cur_rot = RotationVal.BACK
                    self.orientation = Position.RIGHT
                    return True
                elif x1 == x2 and y1 > y2:
                    return False
                else:
                    self.get_logger().error(f"Not a valid next cell value. Restart...")
                    self.stop()
                    exit(1)
            case Position.RIGHT:
                if x1 < x2 and y1 == y2:
                    self.cur_rot = RotationVal.NINETY_RIGHT
                    self.orientation = Position.DOWN
                    return True
                elif x1 > x2 and y1 == y2:
                    self.cur_rot = RotationVal.NINETY_LEFT
                    self.orientation = Position.UP
                    return True
                elif x1 == x2 and y1 < y2:
                    return False
                elif x1 == x2 and y1 > y2:
                    self.cur_rot = RotationVal.BACK
                    self.orientation = Position.LEFT
                    return True
                else:
                    self.get_logger().error(f"Not a valid next cell value. Restart...")
                    self.stop()
                    exit(1)
            case _:
                self.get_logger().error(f"Got Invalid Position for rotation check. Exiting")
                self.stop()
                exit(1)
        

    # --------------------------------------------------------------------------------------------
    # Callback functions

    def proximity_callback(self, message, which):
        self.READY = True
        self.proximities[which] = message.range # Updating the dict values with the current proximity value 
    
    def handle_goal_reached_exploring(self):
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

        
        self.floodfill_state = FloodFillState.SPRINT
        self.get_logger().info("Sprint Mode Activated.")
        self.get_logger().info(f"Best Path: {self.best_path}, Length: {len(self.best_path)}")

    def handle_goal_reached_sprinting(self):
        self.get_logger().info("Goal reached. Done.")

    
    # --------------------------------------------------------------------------------------------
    def check_walls(self):
        return WallState((int(self.proximities['left']>0), int(self.proximities['center']>0), int(self.proximities['right']>0), 0))


    # wall detection and decision making
    def detect_walls(self):
        # walls_robot = WallState((int(0<self.proximities['left']<0.1), int(self.proximities['center']>0), int(0<self.proximities['right']<0.1), int(self.proximities['left_back']>0 and self.proximities['right_back']>0)))
        walls_robot = self.check_walls()
        walls = WallState(self.rotate(walls_robot.value, self.initial_rotation))
        angle = 0
        match(self.orientation):
            case Position.DOWN:
                pass
            case Position.UP:
                angle = np.pi
            case Position.LEFT:
                angle = -np.pi/2
            case Position.RIGHT:
                angle = np.pi/2
            case _:
                self.get_logger().error(f"Aborting, wrong angle")
                exit(1)
        # self.wall_state_global = WallState(self.rotate(walls.value, self.current_pose[2]))
        self.wall_state_global = WallState(self.rotate(walls.value, angle))
        self.get_logger().info(f"WALLS: {walls_robot}")
        self.get_logger().info(f"WALLS: {self.proximities}")
        self.get_logger().info(f"Wall Global: {self.wall_state_global}")
        self.get_logger().info(f"WALLS: {walls}")
        self.get_logger().info(f"Current Movement Global: {self.orientation}")
        self.get_logger().info(f"Current Pose: {self.current_pose}")
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
            max_iters = 1000
            iters = 0

            while not len(queue) == 0:
                iters += 1
                if iters > max_iters:
                    self.get_logger().error("The Maze is not solvable.")
                    self.stop()
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

    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose[1] - current_pose[1], goal_pose[0] - current_pose[0])

    
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
