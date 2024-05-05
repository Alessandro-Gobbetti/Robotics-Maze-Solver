import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, LaserScan, Imu

from enum import Enum
import sys



class WallState(Enum):
    #zero wall
    ZERO = (0,0,0,0)
    #one wall
    FRONT = (1,0,0,0)
    LEFT = (0,1,0,0)
    RIGHT = (0,0,1,0)
    BACK = (0,0,0,1)
    #two walls
    FRONT_LEFT = (1,1,0,0)
    RIGHT_LEFT = (0,1,1,0)
    BACK_LEFT = (0,1,0,1)    
    FRONT_RIGHT = (1,0,1,0)
    BACK_RIGHT = (0,0,1,1)
    FRONT_BACK = (1,0,0,1)
    #three walls
    FRONT_LEFT_RIGHT = (1,1,1,0)
    RIGHT_LEFT_BACK = (0,1,1,1)
    BACK_LEFT_FRONT = (1,1,0,1)   
    FRONT_RIGHT_BACK = (1,0,1,1)
    #four walls
    FRONT_LEFT_RIGHT_BACK = (1,1,1,1)   

class ThymioState(Enum):
    DETECT = 1
    DECIDE = 2
    WALK = 3
    DONE = 4

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.odom_pose = None
        self.odom_velocity = None
        self.pose2d = None
        self.center_prox = -1
        self.left_prox = -1
        self.right_prox = -1
        self.rear_left_prox = -1
        self.rear_right_prox = -1
        self.wall_state = None
        self.mode = ThymioState.WALK
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.

        #Create a subscriber to topic proximity, which will call proximity_callback every time a message is received
        self.center_prox_subscriber = self.create_subscription(Range, 'proximity/center', self.center_prox_callback, 10)
        self.left_prox_subscriber = self.create_subscription(Range, 'proximity/left', self.left_prox_callback, 10)
        self.right_prox_subscriber = self.create_subscription(Range, 'proximity/right', self.right_prox_callback, 10)
        self.rear_left_prox_subscriber = self.create_subscription(Range, 'proximity/rear_left', self.rear_left_prox_callback, 10)
        self.rear_right_prox_subscriber = self.create_subscription(Range, 'proximity/rear_right', self.rear_right_prox_callback, 10) 

        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 10) 
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        #self.get_logger().info(
        #    "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        #)
    
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

    def imu_callback(self,msg):
         self.get_logger().info(
            "imu: received imu: {}".format(msg),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )       



    def center_prox_callback(self, msg):
        self.center_prox = round(msg.range,2)
        self.get_logger().info(
            "center_prox: received center proximity : {:.2f}".format(self.center_prox),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        
    def left_prox_callback(self, msg):
        self.left_prox = round(msg.range,2)
        self.get_logger().info(
            "left_prox: received left proximity : {:.2f}".format(self.left_prox),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )        
        
    def right_prox_callback(self, msg):
        self.right_prox = round(msg.range,2)
        self.get_logger().info(
            "right_prox: received right proximity : {:.2f}".format(self.right_prox),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )                       
    def rear_left_prox_callback(self, msg):
        self.rear_left_prox = round(msg.range,2)
        self.get_logger().info(
            "rear_left_prox: received rear left proximity : {:.2f}".format(self.rear_left_prox),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

    def rear_right_prox_callback(self, msg):
        self.rear_right_prox = round(msg.range,2)
        self.get_logger().info(
            "rear_right_prox: received rear right proximity : {:.2f}".format(self.rear_right_prox),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

    def update_callback(self):     
        if self.mode == ThymioState.DETECT:
            self.update_detect()
        elif self.mode == ThymioState.DECIDE:
            self.update_decide()
        elif self.mode == ThymioState.WALK:
        # Let's just set some hard-coded velocities in this example
            cmd_vel = Twist() 
            cmd_vel.linear.x  = 0.2 # [m/s]
            cmd_vel.angular.z = 0.0 # [rad/s]
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        elif self.mode == ThymioState.DONE:
            return 


    def update_detect(self):
        self.wall_state = WallState((int(self.center_prox>0),int(self.left_prox>0),int(self.right_prox>0),int(self.rear_left_prox>0 and self.rear_right_prox>0)))
        print(self.wall_state)

    def update_decide(self):
        pass

def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
