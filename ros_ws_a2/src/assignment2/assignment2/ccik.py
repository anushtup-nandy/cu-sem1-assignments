#!/usr/bin/env python3

import math
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from custom_msg.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import transforms3d
import transforms3d._gohlketransforms as tf
from threading import Thread, Lock

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(Node):
    def __init__(self):
        super().__init__('ccik')
    #Load robot from parameter server
        # self.robot = URDF.from_parameter_server()
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        # print(robot_desription_text)
        self.robot = URDF.from_xml_string(robot_desription_text)

    #Subscribe to current joint state of the robot
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.get_joint_state, 10)

    #This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

    #This is a mutex
        self.mutex = Lock()

    #Subscribers and publishers for for cartesian control
        self.cartesian_command_sub = self.create_subscription(
            CartesianCommand, '/cartesian_command', self.get_cartesian_command, 10)
        self.velocity_pub = self.create_publisher(JointState, '/joint_velocities', 10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        self.ik_command_sub = self.create_subscription(
            Transform, '/ik_command', self.get_ik_command, 10)
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command: CartesianCommand):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        # Implement your code here
        #--------------------------------------------------------------------------
        # Create message to publish
        self.joint_velocity_msg.name = self.joint_names
        
        # Get current joint positions and calculate current pose
        joint_transforms, b_T_ee = self.forward_kinematics(self.q_current)
        
        # Get target transform from command
        target = np.identity(4)
        target[0:3, 3] = [command.x_target.translation.x, 
                        command.x_target.translation.y, 
                        command.x_target.translation.z]
        q_target = [command.x_target.rotation.x,
                    command.x_target.rotation.y,
                    command.x_target.rotation.z,
                    command.x_target.rotation.w]
        target[0:3, 0:3] = transforms3d.quaternions.quat2mat(q_target)
        
        # Calculate pose error
        error_transform = np.linalg.inv(b_T_ee) @ target
        
        # Extract position error
        pos_error = error_transform[0:3, 3]
        
        # Extract rotation error using angle-axis representation
        angle, axis = self.rotation_from_matrix(error_transform[0:3, 0:3])
        rot_error = axis * angle
        
        # Combine into single error vector
        error = np.zeros(6)
        error[0:3] = pos_error  # Position error
        error[3:6] = rot_error  # Rotation error
        
        # Get Jacobian at current position
        J = self.get_jacobian(b_T_ee, joint_transforms)
        
        # Calculate joint velocities using pseudo-inverse
        # Using proportional gain of 1.0 (can be increased if needed)
        p_gain = 1.0
        v_cart = p_gain * error
        
        if command.secondary_objective:
            # Calculate nullspace projection matrix
            J_pinv = np.linalg.pinv(J)
            null_proj = np.eye(self.num_joints) - J_pinv @ J
            
            # Calculate secondary objective velocity
            # Only affects first joint
            q0_error = command.q0_target - self.q_current[0]
            v_null = np.zeros(self.num_joints)
            v_null[0] = q0_error * 3.0  # gain of 3 for secondary objective
            
            # Combine primary and secondary objectives
            dq = J_pinv @ v_cart + null_proj @ v_null
        else:
            # Only primary objective
            J_pinv = np.linalg.pinv(J)
            dq = J_pinv @ v_cart
        
        # Publish joint velocities
        self.joint_velocity_msg.velocity = dq.tolist()
        self.velocity_pub.publish(self.joint_velocity_msg)

        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = np.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        # Implement your code here
        #--------------------------------------------------------------------------
        J = np.zeros((6, self.num_joints))
    
        # Get the position of end effector
        p_ee = b_T_ee[:3, 3]
        
        # Calculate Jacobian column for each joint
        for i in range(self.num_joints):
            # Get current joint transform
            current_transform = joint_transforms[i]
            
            # Extract rotation matrix and position from transform
            R = current_transform[:3, :3]
            p = current_transform[:3, 3]
            
            # Get joint axis vector and rotate it by joint transform
            joint_axis = np.array(self.joint_axes[i])
            z = R @ joint_axis
            
            # Calculate linear velocity component (cross product)
            p_diff = p_ee - p
            J[:3, i] = np.cross(z, p_diff)
            
            # Angular velocity component is just the rotated joint axis
            J[3:, i] = z
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        # Implement your code here
        #-----------------------------------------------,---------------------------

        # Create message to publish
        self.joint_command_msg.name = self.joint_names
        
        # Get target transform from command
        target = np.identity(4)
        target[0:3, 3] = [command.translation.x, command.translation.y, command.translation.z]
        q_target = [command.rotation.x, command.rotation.y, command.rotation.z, command.rotation.w]
        target[0:3, 0:3] = transforms3d.quaternions.quat2mat(q_target)
        
        max_attempts = 3
        timeout = 10.0  # seconds
        
        for attempt in range(max_attempts):
            # Initialize solution with current joint positions
            q_sol = np.array(self.q_current)
            start_time = time.time()
            
            while time.time() - start_time < timeout/max_attempts:
                # Get current end effector pose
                joint_transforms, b_T_ee = self.forward_kinematics(q_sol)
                
                # Check if we've reached the target
                error_transform = np.linalg.inv(b_T_ee) @ target
                pos_error = np.linalg.norm(error_transform[0:3, 3])
                
                # Get rotation error using angle-axis representation
                angle, axis = self.rotation_from_matrix(error_transform[0:3, 0:3])
                rot_error = abs(angle)
                
                # Check if we're close enough
                if pos_error < 1e-3 and rot_error < 1e-3:
                    # Success! Publish the solution
                    self.joint_command_msg.position = q_sol.tolist()
                    self.joint_command_pub.publish(self.joint_command_msg)
                    self.mutex.release()
                    return
                
                # Calculate error vector
                error = np.zeros(6)
                error[0:3] = error_transform[0:3, 3]  # Position error
                error[3:6] = axis * angle  # Rotation error
                
                # Get Jacobian at current position
                J = self.get_jacobian(b_T_ee, joint_transforms)
                
                # Calculate joint velocities using pseudo-inverse
                J_pinv = np.linalg.pinv(J)
                dq = J_pinv @ error
                
                # Update solution (with small step size for stability)
                step_size = 0.5
                q_sol += step_size * dq
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = np.array(matrix, dtype=np.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = np.linalg.eig(R33.T)
        i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = np.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = np.linalg.eig(R)
        i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (np.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each link of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = np.dot(tf.translation_matrix(joint.origin.xyz), tf.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], 'rxyz'))
            T = np.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.rotation_matrix(joint_values[q_index], np.asarray(joint.axis))
                T = np.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


def main(args = None):
    rclpy.init()
    ccik = CCIK()
    rclpy.spin(ccik)
    ccik.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
