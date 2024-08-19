import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create a subscriber to the camera image topic
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',  # Change to your actual camera topic
            self.image_callback,
            10)
        
        # Create a subscriber to the LaserScan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',  # Change to your actual LaserScan topic
            self.scan_callback,
            10)

        # Parameters
        self.stop_distance = 0.6  # Distance in meters to stop from any object
        self.rotation_speed = 0.5  # Rotation speed
        self.linear_speed = 0.5   # Forward speed

        # Timer for controlling rotation and movement
        self.timer = self.create_timer(0.1, self.control_robot)
        
        # Variables for tracking the robot's state
        self.found_sphere = False
        self.target_x = None
        self.laser_scan_ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Process the image to find the yellow sphere
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        
        # Find contours of the yellow objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate the center of the bounding box
            self.target_x = x + w / 2
            self.found_sphere = True
        else:
            self.found_sphere = False

    def scan_callback(self, msg):
        self.laser_scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

    def control_robot(self):
        # Create a Twist message
        cmd = Twist()
        
        # Calculate the indices for -45 to 45 degrees range
        if self.laser_scan_ranges:
            '''
            num_readings = len(self.laser_scan_ranges)
            
            # Convert -45 to 45 degrees into radians
            range_min_angle = np.deg2rad(-45)
            range_max_angle = np.deg2rad(45)

            # Calculate the indices corresponding to -45 and 45 degrees
            min_index = int((range_min_angle - self.angle_min) / self.angle_increment)
            max_index = int((range_max_angle - self.angle_min) / self.angle_increment)
            
            # Handle circular indexing: wrap around for negative angles
            if min_index < 0:
                min_index = num_readings + min_index

            # Ensure max_index is within bounds
            max_index = min(num_readings - 1, max_index)

            # Combine the ranges from the front and the wrapped-around part
            if min_index > max_index:
                # This happens when the range crosses the 0-degree line, so we take two slices
                relevant_ranges = self.laser_scan_ranges[min_index:] + self.laser_scan_ranges[:max_index + 1]
            else:
                relevant_ranges = self.laser_scan_ranges[min_index:max_index + 1]
            
            # Check the minimum distance in the relevant range
            min_distance = min(relevant_ranges)

            # Print the distance for debugging
            self.get_logger().info(f'Distance to obstacle: {min_distance:.2f} meters')
            '''
            
            min_distance = min(self.laser_scan_ranges)


            if min_distance < self.stop_distance and self.found_sphere == True:
                # Stop if too close to any object
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
            elif self.found_sphere:
                if self.target_x is not None:
                    # Calculate the center of the image
                    center_x = 1920 / 2
                    
                    # Rotate to align with the target
                    if self.target_x < center_x -200:  # Allowing some tolerance
                        cmd.angular.z = float(self.rotation_speed)
                    elif self.target_x > center_x + 200:
                        cmd.angular.z = float(-self.rotation_speed)
                    else:
                        cmd.angular.z = 0.0
                    
                    # Move forward when aligned
                    cmd.linear.x = float(self.linear_speed)
                else:
                    cmd.angular.z = float(self.rotation_speed)
                    cmd.linear.x = 0.0
            else:
                # Rotate in place if no sphere found
                cmd.angular.z = float(self.rotation_speed)
                cmd.linear.x = 0.0

        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







