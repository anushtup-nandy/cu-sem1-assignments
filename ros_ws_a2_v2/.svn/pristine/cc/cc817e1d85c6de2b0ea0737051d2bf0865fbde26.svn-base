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
        '''
        Args:
            command: CartesianCommand message containing target pose and secondary objective

        How it works:
        1. queries the current state and extracts the target pose
        2. find the desired transform and the error
        3. find and extract the position and rotation error
        4. compute the joint velocity using the jacobian function written earlier
        5. compute and resolve the null space optimization (secondary objective--> only needed if robot has a redundancy)
            5.1 find the error in the position of the set base position and the current base position
            5.2 set all intial velocities to 0
            5.3 Assign a velocity to the first joint (q_dot_sec[0]). 
                This velocity is proportional to the error (q0_error), scaled by a factor of 3.0 (proportional gain)
            5.4 Compute the pseudoinverse of J with some damping (regularizes it)
            5.5 find the null-space projection --> basically where joint velocities do not affect the end-effectors motion
            5.6 Update the joint velocties
        '''
        joint_transforms, b_T_ee_current = self.forward_kinematics(self.q_current)

        x_tar = command.x_target
        d_pos = np.array([x_tar.translation.x, x_tar.translation.y, x_tar.translation.z])
        quat = np.array([x_tar.rotation.w, x_tar.rotation.x, x_tar.rotation.y, x_tar.rotation.z])
        des_trans = transforms3d.affines.compose(d_pos, 
                                               transforms3d.quaternions.quat2mat(quat), 
                                               (1, 1, 1))        

        delta_trans = np.dot(np.linalg.inv(b_T_ee_current), des_trans)

        p_delta, rot_delta_mat, _, _ = transforms3d.affines.decompose(delta_trans)
        rot_delta_angle, rot_delta_ax = self.rotation_from_matrix(rot_delta_mat)
        rot_delta = rot_delta_angle * rot_delta_ax

        delta_cart = np.concatenate([p_delta, rot_delta])
        J = self.get_jacobian(b_T_ee_current, joint_transforms)
        damping = 1.0e-2
        joint_v = np.dot(np.linalg.pinv(J, damping), delta_cart)

        if command.secondary_objective:
            q0_error = command.q0_target - self.q_current[0]
            q_dot_sec = np.zeros(self.num_joints)
            q_dot_sec[0] = 3.0 * q0_error
            
            J_pinv = np.linalg.pinv(J, damping)
            J_null = np.eye(self.num_joints) - np.dot(J_pinv, J)
            joint_v += np.dot(J_null, q_dot_sec)

        self.joint_velocity_msg.name = self.joint_names
        self.joint_velocity_msg.velocity = joint_v.tolist()
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
        '''
        Args:
            b_T_ee: 4x4 transformation matrix from base to end-effector
            joint_transforms: List of 4x4 transformation matrices for each joint
        
        Return:
            J: 6xn Jacobian matrix mapping joint velocities to end-effector velocities
        How it works:
        1. Find the relative transforms (bTee = bTj * jTee)
        2. extract the position and rotation components
        3. compute the skew symmetric matrix
        4. assemble Vj
        5. Get the correct axis (generally z, but still checking!)
        6. Assemble the entire Jacobian 
        '''         
        for j in range(self.num_joints):
            j_T_ee = np.dot(np.linalg.inv(joint_transforms[j]), b_T_ee) 
            ee_T_j = np.linalg.inv(j_T_ee)
            j_t_ee = j_T_ee[:3, 3] 
            ee_R_j = ee_T_j[:3, :3]  

            skew_prod = np.dot(ee_R_j, np.array([
            [0, -j_t_ee[2], j_t_ee[1]],
            [j_t_ee[2], 0, -j_t_ee[0]],
            [-j_t_ee[1], j_t_ee[0], 0]
            ]))

            V_j = np.block([
                [ee_R_j, -skew_prod],
                [np.zeros((3, 3)), ee_R_j]
            ])

            axis_index = {
            (0.0, 0.0, 1.0): 5,
            (0.0, 1.0, 0.0): 4,
            (1.0, 0.0, 0.0): 3
             }.get(tuple(self.joint_axes[j]), 5) 
        
            J[:, j] = V_j[:, axis_index]
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
        '''
        args: command has the target and end-effector pose!
        How the numerical IK works:
        1. extracts the target pose fist
            1.1 builds it into a transformation matrix
        2. The parameters are initialized (IK solver, such as the iterations it should run for, the attempts, etc)
        3. Initializes the timer to find the solution and to  terminate if it can't (as specified in the handout)
            3.1 also initializes the best solution --> Tracking best solution found
        4. Initializes random configuration (qc = RAND())
        5. Iterates over the maximum permitted iterations
            5.1 performs FK
            5.2 finds the error in the pose of the robot
            5.3 if the error is lower than the the previous one then update error
            5.4 if convergence is upto the threshold, then terminate!
            5.5 else update the joint information
        '''
        x_tar = command
        target_pos = np.array([x_tar.translation.x, x_tar.translation.y, x_tar.translation.z])
        target_quat = np.array([x_tar.rotation.w, x_tar.rotation.x, x_tar.rotation.y, x_tar.rotation.z])
        xd = transforms3d.affines.compose(target_pos, 
                                        transforms3d.quaternions.quat2mat(target_quat), 
                                        (1, 1, 1))
        
        # IK solver parameters
        params = {
            'max_iterations': 100,
            'max_attempts': 5,
            'threshold': 1e-3,
            'time_limit': 2.0,
            'damping': 1.0e-2,
            'step_size': 0.5
        }
        
        start_time = time.time()
        best_solution = None
        best_error = float('inf')
        
        for attempt in range(params['max_attempts']):
            if time.time() - start_time > params['time_limit']:
                break
    
            qc = np.random.uniform(-np.pi, np.pi, self.num_joints)
            
            for iteration in range(params['max_iterations']):
            
                joint_transforms, xc = self.forward_kinematics(qc)

                delta_trans = np.dot(np.linalg.inv(xc), xd)
                p_delta, rot_delta_mat, _, _ = transforms3d.affines.decompose(delta_trans)
                rot_delta_angle, rot_delta_ax = self.rotation_from_matrix(rot_delta_mat)
                rot_delta = rot_delta_angle * rot_delta_ax
                
                delta_x = np.concatenate([p_delta, rot_delta])
                error_magnitude = np.linalg.norm(delta_x)
                
                if error_magnitude < best_error:
                    best_error = error_magnitude
                    best_solution = qc.copy()
                
                if error_magnitude < params['threshold']:
                    break
                
                J = self.get_jacobian(xc, joint_transforms)
                delta_q = np.dot(np.linalg.pinv(J, params['damping']), delta_x)
                qc = qc + params['step_size'] * delta_q
                
                qc = np.mod(qc + np.pi, 2 * np.pi) - np.pi
        
        # Publish results
        self.joint_command_msg.name = self.joint_names
        self.joint_command_msg.position = (best_solution if best_solution is not None 
                                         else [0.0] * self.num_joints).tolist()
        self.joint_command_pub.publish(self.joint_command_msg)
        
        
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
