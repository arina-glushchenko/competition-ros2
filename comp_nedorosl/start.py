from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from .pid import PID
from .sign_recognition import TrafficSignDetector
import math
import time
import threading


class Starter(Node):
    def __init__(self):
        super().__init__('publisher')

        # Publishers for controlling robot state and movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finish_publisher = self.create_publisher(String, 'robot_finish', 1)
        self.stop_publisher = self.create_publisher(String, "robot_stop", 1)

        # Subscriptions for various sensor inputs
        self.create_subscription(Empty, "robot_start", self.empty_listener_callback, 5)
        self.create_subscription(Image, "/color/image", self.image_callback, 1)
        self.create_subscription(Image, "/color/image", self.traffic_light_callback, 1)
        self.create_subscription(LaserScan, "/scan", self.laser_callback, 1)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Timer for periodic updates
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize main components
        self.pid = PID(self.timer_period)
        model_path = '/home/arina/study/robotics/ros2_ws/src/comp_nedorosl/comp_nedorosl/best.pt'
        self.sign_detector = TrafficSignDetector(model_path)

        # State variables
        self.is_started = False
        self.state = 'none'
        self.cvBridge = CvBridge()
        self.latest_laser_data = None
        self.robot_pose = None
        self.robot_yaw = None
        self.detected_sign = None
        self.turn_count = 0

        # Threads for parallel processing
        self.detect_thread = None
        self.pid_thread = None

    def image_callback(self, msg):
        """
        Callback function for processing camera image input.
        It handles both traffic sign detection and PID control in separate threads.
        """
        if not self.is_started:
            return

        # Convert the ROS image message to an OpenCV image
        image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Start traffic sign detection in a separate thread
        if self.detect_thread is None or not self.detect_thread.is_alive():
            self.detect_thread = threading.Thread(target=self.detect_traffic_sign, args=(image,))
            self.detect_thread.start()

        # Start PID control in a separate thread
        if self.pid_thread is None or not self.pid_thread.is_alive():
            self.pid_thread = threading.Thread(target=self.pid_control, args=(image,))
            self.pid_thread.start()

    def detect_traffic_sign(self, image):
        """
        Detects traffic signs from the given image and updates the robot's state accordingly.
        """
        detected_class = self.sign_detector.recognition(image)
        if detected_class in [3, 5]:  # Example: 3=left turn, 5=right turn
            self.detected_sign = detected_class
            self.state = 'traffic_intersection'
        else:
            self.state = 'none'

    def pid_control(self, image):
        """
        Handles PID control logic based on the robot's current state.
        """
        if self.state == 'none':
            self.pid.calc_error(image)

    def intersection(self):
        """
        Executes maneuvers at traffic intersections based on the detected traffic sign.
        """
        if not self.detected_sign:
            return

        # Reset yaw before the first turn
        if self.turn_count == 0 and self.robot_yaw is not None:
            self.robot_yaw = 0.0

        cmd_vel = Twist()

        # Move forward slightly before making a turn
        cmd_vel.linear.x = 0.1
        self.publisher.publish(cmd_vel)
        time.sleep(1.0)

        # Perform the turn based on the detected traffic sign
        if self.detected_sign == 3 and self.turn_count <= 1:  # Turn left
            cmd_vel.angular.z = 0.7
            self.turn_count += 1
        elif self.detected_sign == 5 and self.turn_count <= 2:  # Turn right
            cmd_vel.angular.z = -0.7
            self.turn_count += 1

        # Stop turning
        cmd_vel.linear.x = 0.0
        self.publisher.publish(cmd_vel)
        time.sleep(0.7)

        # Stop the robot after completing the turn
        cmd_vel.angular.z = 0.0
        self.publisher.publish(cmd_vel)

        # Reset state for the next operation
        self.detected_sign = None
        self.state = 'none'

    def timer_callback(self):
        """
        Periodic callback for updating the robot's state and publishing commands.
        """
        if not self.is_started:
            return

        if self.state == 'traffic_intersection':
            self.intersection()
        elif self.state == 'none':
            new_vel = self.pid.update_error()
            self.publisher.publish(new_vel)

        # Publish start or stop signal based on the robot's state
        msg = String()
        msg.data = 'stop' if self.state != 'none' else 'start'
        self.stop_publisher.publish(msg)

    def traffic_light_callback(self, msg):
        """
        Callback for processing traffic light detection.
        Example condition checks for green light.
        """
        frame = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame[300, 600][1] == 109:  # Example condition for green light
            self.is_started = True

    def empty_listener_callback(self, msg):
        """
        Callback to start the robot upon receiving a signal.
        """
        self.is_started = True

    def laser_callback(self, msg):
        """
        Callback for processing laser scan data.
        """
        self.latest_laser_data = msg

    def odom_callback(self, msg):
        """
        Callback for processing odometry data.
        """
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def get_yaw_from_quaternion(self, orientation):
        """
        Helper function to extract yaw angle from a quaternion.
        """
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.
    """
    rclpy.init(args=args)
    starting = Starter()
    rclpy.spin(starting)
    rclpy.shutdown()
