import rclpy
import sys
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class PID:
    def __init__(self, timer_period):
        """
        Initializes the PID controller.

        :param timer_period: The time interval (in seconds) between PID updates.
        """
        self.timer_period = timer_period
        self.error = [0, 0]  # List to store current and previous errors
        self.curr_time = 0  # Keeps track of elapsed time
        self.velocity = 0.11  # Constant forward velocity

        # Error constraints
        self.max_error = 100  # Maximum allowable error
        self.min_error = -100  # Minimum allowable error

    def calc_error(self, img):
        """
        Calculates the lateral error based on detected lane lines in the input image.

        :param img: Input image (BGR format).
        """
        # Convert image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define color ranges for detecting white and yellow lane markings
        sensitivity = 19
        lower_white = np.array([0, 0, 255 - sensitivity])
        upper_white = np.array([255, sensitivity, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create masks for white and yellow regions
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.bitwise_or(mask_white, mask_yellow)

        # Crop the bottom part of the mask for lane detection
        cropped_mask = mask[int(mask.shape[0] / 3 * 2):, :]

        # Detect edges using the Canny edge detection method
        edges = cv2.Canny(cropped_mask, 50, 150)

        # Apply Sobel filter to detect gradients along the x-axis
        sobelx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)
        abs_sobelx = cv2.convertScaleAbs(sobelx)

        # Split the image into left and right halves for analysis
        mid_point = cropped_mask.shape[1] // 2
        left_side = abs_sobelx[:, :mid_point]
        right_side = abs_sobelx[:, mid_point:]

        # Find the first non-zero pixels (inner edges) in each row
        left_inner_edge = np.argmax(left_side > 0, axis=1)
        right_inner_edge = mid_point + np.argmax(right_side[::-1] > 0, axis=1)

        # Calculate the average position of the detected inner edges
        valid_left = left_inner_edge[left_inner_edge > 0]
        valid_right = right_inner_edge[right_inner_edge > 0]

        if len(valid_left) > 0 and len(valid_right) > 0:
            left_avg = np.mean(valid_left)
            right_avg = np.mean(valid_right)
            fpt = (left_avg + right_avg) / 2  # Midpoint between left and right edges
        else:
            fpt = cropped_mask.shape[1] / 2  # Default to the center if no edges are detected

        # Compute the lateral error (distance from the center of the image)
        self.error.append(cropped_mask.shape[1] / 2 - fpt)

        # Clamp the error within the allowed range
        if self.error[-1] > self.max_error:
            self.error[-1] = self.max_error
        elif self.error[-1] < self.min_error:
            self.error[-1] = self.min_error

    def update_error(self):
        """
        Computes the control command (linear and angular velocity) based on the PID algorithm.

        :return: A Twist message containing linear and angular velocity commands.
        """
        self.curr_time += self.timer_period  # Update elapsed time
        cmd_vel = Twist()  # Create a new Twist message
        cmd_vel.linear.x = self.velocity  # Set forward velocity

        # Adaptive PID coefficients
        if abs(self.error[-1]) > 30:  # Higher error requires more aggressive correction
            Kp = 0.015  # Proportional gain
            Kd = 0.6    # Derivative gain
        else:
            Kp = 0.015
            Kd = 0.6

        Ki = 0.0  # Integral gain is disabled in this implementation

        # Compute the angular velocity using the PID formula
        cmd_vel.angular.z = float(
            Kp * self.error[-1]  # Proportional term
            + Ki * np.sum(np.array(self.error) * self.timer_period)  # Integral term
            + Kd * (self.error[-1] - self.error[-2]) / self.timer_period  # Derivative term
        )

        return cmd_vel
