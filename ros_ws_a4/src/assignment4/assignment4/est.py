#!/usr/bin/env python3

# Columbia Engineering
# MECS 4603 - Fall 2023

import math
import numpy
import time

import rclpy
from rclpy.node import Node

from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import SensorData

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')

        # Publisher to publish state estimate
        self.pub_est = self.create_publisher(RobotPose, "/robot_pose_estimate", 1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        # Measurement noise covariance
        self.W = numpy.array([[0.1, 0],
                            [0, 0.05]])

        self.step_size = 0.05

        # Subscribe to command input and sensory output of robot
        self.sensor_sub = self.create_subscription(SensorData, "/sensor_data", self.sensor_callback, 1)
    
    def normalize_angle(self, angle):
        '''
        Args: angle (float)
        Output: normalized angle (float)
        Normalizes an angle to be between -π and π
        Uses a while loop to repeatedly add or subtract 2π until the angle is in the desired range
        '''
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_jacobian_F(self, vel_trans, vel_ang, theta):
        '''
        Args: vel_trans (float), vel_ang (float), theta (float)
        Output: F (3x3 numpy array)
        - Computes the Jacobian matrix of the state transition function
        - Creates a 3x3 identity matrix as base
        - Updates elements F[0,2] and F[1,2] based on velocity and theta
        - Used in the prediction step of the EKF
        '''
        dt = self.step_size
        F = numpy.eye(3)
        F[0,2] = -dt * vel_trans * math.sin(theta)
        F[1,2] = dt * vel_trans * math.cos(theta)
        return F

    def compute_jacobian_H(self, x, y, theta, landmark_x, landmark_y):
        '''
        Args: x, y, theta, landmark_x, landmark_y (all floats)
        Output: H (2x3 numpy array)
        - Computes the Jacobian matrix of the measurement function
        - Calculates derivatives for range and bearing measurements
        - Returns H matrix containing partial derivatives for the measurement model
        '''
        dx = landmark_x - x
        dy = landmark_y - y
        d2 = dx*dx + dy*dy
        d = math.sqrt(d2)
        
        H = numpy.zeros((2,3))
        
        # Derivatives for range measurement
        H[0,0] = -dx/d
        H[0,1] = -dy/d
        H[0,2] = 0
        
        # Derivatives for bearing measurement
        H[1,0] = dy/d2
        H[1,1] = -dx/d2
        H[1,2] = -1
        
        return H

    def predict_measurement(self, x, y, theta, landmark_x, landmark_y):
        '''
        Args: x, y, theta, landmark_x, landmark_y (all floats)
        Output: 2x1 numpy array [range_pred, bearing_pred]

        - Predicts the expected range and bearing measurements to a landmark
        - Calculates the Euclidean distance for range
        - Calculates the relative bearing using atan2 and normalizes it
        '''
        dx = landmark_x - x
        dy = landmark_y - y
        
        range_pred = math.sqrt(dx*dx + dy*dy)
        bearing_pred = self.normalize_angle(math.atan2(dy, dx) - theta)
        
        return numpy.array([[range_pred], [bearing_pred]]) 
    
    def estimate(self, sens: SensorData):
        '''This function gets called every time the robot publishes its control 
        input and sensory output. You must make use of what you know about 
        extended Kalman filters to come up with an estimate of the current
        state of the robot and covariance matrix. The SensorData message 
        contains fields 'vel_trans' and 'vel_ang' for the commanded 
        translational and rotational velocity respectively. Furthermore, 
        it contains a list 'readings' of the landmarks the robot can currently
        observe

        Args:
            sens: incoming sensor message
        - Implements the complete Extended Kalman Filter algorithm:
        - Performs prediction step using motion model
        - Updates state and covariance using process model
        '''
        # Prediction step
        dt = self.step_size
        theta = float(self.x[2])
        
        # Predict state
        self.x[0] += dt * sens.vel_trans * math.cos(theta)
        self.x[1] += dt * sens.vel_trans * math.sin(theta)
        self.x[2] += dt * sens.vel_ang
        self.x[2] = self.normalize_angle(float(self.x[2]))
        
        # Jacobian F
        F = self.compute_jacobian_F(sens.vel_trans, sens.vel_ang, theta)
        
        # covariance
        self.P = F @ self.P @ F.T + self.V
        
        # Update step 
        for reading in sens.readings:
            landmark_x = reading.landmark.x
            landmark_y = reading.landmark.y
            
            # Skip if robot is too close to landmark
            dx = landmark_x - float(self.x[0])
            dy = landmark_y - float(self.x[1])
            if math.sqrt(dx*dx + dy*dy) < 0.1:
                continue
                
            # Compute measurement Jacobian
            H = self.compute_jacobian_H(
                float(self.x[0]), float(self.x[1]), float(self.x[2]),
                landmark_x, landmark_y
            )
            
            # Predict measurement
            z_pred = self.predict_measurement(
                float(self.x[0]), float(self.x[1]), float(self.x[2]),
                landmark_x, landmark_y
            )
            
            # Compute innovation
            z_actual = numpy.array([[reading.range], [reading.bearing]])
            innovation = z_actual - z_pred
            
            # Normalize bearing innovation to [-pi, pi]
            innovation[1] = self.normalize_angle(float(innovation[1]))
            
            # Compute Kalman gain
            S = H @ self.P @ H.T + self.W
            K = self.P @ H.T @ numpy.linalg.inv(S)
            
            # Update state and covariance
            self.x += K @ innovation
            self.P = (numpy.eye(3) - K @ H) @ self.P
            
            # Normalize theta in state vector
            self.x[2] = self.normalize_angle(float(self.x[2]))
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = float(self.x[0])
        est_msg.pose.y = float(self.x[1])
        est_msg.pose.theta = float(self.x[2])
        self.pub_est.publish(est_msg)

def main(args=None):
    rclpy.init(args=args)   
    est = Estimator()
    rclpy.spin(est)
                
if __name__ == '__main__':
   main()

 
