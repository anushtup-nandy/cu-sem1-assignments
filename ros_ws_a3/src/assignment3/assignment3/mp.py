#!/usr/bin/env python3
import numpy
import random
import sys

import moveit_msgs.msg
import moveit_msgs.srv
import rclpy
from rclpy.node import Node
import rclpy.duration
import transforms3d._gohlketransforms as tf
import transforms3d
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import copy
import math

def convert_to_message(T):
    t = Pose()
    position, Rot, _, _ = transforms3d.affines.decompose(T)
    orientation = transforms3d.quaternions.mat2quat(Rot)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]        
    return t

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        #Loads the robot model, which contains the robot's kinematics information
        self.ee_goal = None
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        #Loads the robot model, which contains the robot's kinematics information
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)
        self.base = self.robot.get_root()
        self.get_joint_info()


        self.service_cb_group1 = MutuallyExclusiveCallbackGroup()
        self.service_cb_group2 = MutuallyExclusiveCallbackGroup()
        self.q_current = []

        # Wait for moveit IK service
        self.ik_service = self.create_client(moveit_msgs.srv.GetPositionIK, '/compute_ik', callback_group=self.service_cb_group1)
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service ready')

        # Wait for validity check service
        self.state_valid_service = self.create_client(moveit_msgs.srv.GetStateValidity, '/check_state_validity',
                                                      callback_group=self.service_cb_group2)
        while not self.state_valid_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state validity service...')
        self.get_logger().info('State validity service ready')

        # MoveIt parameter
        self.group_name = 'arm'
        self.get_logger().info(f'child map: \n{self.robot.child_map}')

        #Subscribe to topics
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.get_joint_state, 10)
        self.goal_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_goal = self.create_subscription(Transform, '/motion_planning_goal', self.motion_planning_cb, 2,
                                                 callback_group=self.goal_cb_group)
        self.current_obstacle = "NONE"
        self.sub_obs = self.create_subscription(String, '/obstacle', self.get_obstacle, 10)

        #Set up publisher
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.motion_planning_timer, callback_group=self.timer_cb_group)

    
    def get_joint_state(self, msg):
        '''This callback provides you with the current joint positions of the robot 
        in member variable q_current.
        '''
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    
    def get_obstacle(self, msg):
        '''This callback provides you with the name of the current obstacle which
        exists in the RVIZ environment. Options are "None", "Simple", "Hard",
        or "Super". '''
        self.current_obstacle = msg.data

    def motion_planning_cb(self, ee_goal):
        self.get_logger().info("Motion planner goal received.")
        if self.ee_goal is not None:
            self.get_logger().info("Motion planner busy. Please try again later.")
            return
        self.ee_goal = ee_goal

    def motion_planning_timer(self):
        if self.ee_goal is not None:
            self.get_logger().info("Calling motion planner")            
            self.motion_planning(self.ee_goal)
            self.ee_goal = None
            self.get_logger().info("Motion planner done")            
                
   
    def motion_planning(self, ee_goal: Transform): 
        '''Callback function for /motion_planning_goal. This is where you will
        implement your RRT motion planning which is to generate a joint
        trajectory for your manipulator. You are welcome to add other functions
        to this class (i.e. an is_segment_valid" function will likely come in 
        handy multiple times in the motion planning process and it will be 
        easiest to make this a seperate function and then call it from motion
        planning). You may also create trajectory shortcut and trajectory 
        sample functions if you wish, which will also be called from the 
        motion planning function.

        Args: 
            ee_goal: Transform() object describing the desired base to 
            end-effector transformation 
        '''
        
        # TODO: implement motion_planning here
        '''
        1. Goal Transformation Setup:
            - desired position and orientation are extracted from ee_goal
            - position and orientation (quaternion) are made compatible with robot's kinemtics functions.
        2. IK check is performed next which attempts to find joint configurations (joints_goal) that will achieve des_transform. 
            If no solution is found, a warning is logged, and the function exits.
        3. Initialize RRT Tree:
            - setup branch (first one) as a node with current joint configand add it to RRT tree
            - tree grows by adding new branches that incrementally approach joint_goals
            --  max_iter = 10000
                goal_threshold = 0.1  # Radians
                min_goal_bias = 0.1
                max_goal_bias = 0.5
        4. Sampling loop
            - has adaptive goal biasing (which) increases as the robot nears joints_goal --> essentially prioritize goal-directed exploration over random sampling
            - The goal_bias is computed based on the distance between the joints_goal and the last node in tree
            - with some probability goal_bias, joints_goal is selected as the next sample (else a random one is drawn within the joint limits)
            - sampling from random nodes in the tree with a small gaussian noise --> diversify and avoid obstacles
            - Find the closest node inn the tree to "sample" and then validate if path can be extended from this node to sample.
            - check if goal can be reached from this point!
        5. Extract path ad optimize
        6. Create the trajectory and publish topic
        '''

        des_pos = numpy.array([ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z])
        quat = numpy.array([ee_goal.rotation.w, ee_goal.rotation.x, 
                ee_goal.rotation.y, ee_goal.rotation.z])
        des_rot = transforms3d.quaternions.quat2mat(quat)
        des_transform = transforms3d.affines.compose(des_pos, des_rot, (1,1,1))
        
        joints_goal = self.IK(des_transform)
        if not joints_goal:
            self.get_logger().warn("No IK solution found for goal")
            return
        
        # Initialize RRT
        joints_current = self.q_current
        init_branch = RRTBranch(None, joints_current)
        tree = [init_branch]
        
        # RRT parameters
        max_iter = 10000
        goal_threshold = 0.1  # Radians
        min_goal_bias = 0.1
        max_goal_bias = 0.5

        for n in range(max_iter):
            distance_to_goal = numpy.linalg.norm(numpy.array(tree[-1].q) - numpy.array(joints_goal))
            goal_bias = min_goal_bias + (max_goal_bias - min_goal_bias) * numpy.exp(-distance_to_goal)
            

            if random.random() < goal_bias:
                sample = joints_goal
            else:
                random_node = random.choice(tree)
                sample = numpy.array(random_node.q) + numpy.random.normal(0, 0.5, self.num_joints)
                sample = numpy.clip(sample, -math.pi, math.pi)

            nearest_branch = self.find_closest_point_in_tree(tree, sample)
            valid_extension, new_config = self.segment_valid(sample, nearest_branch.q)
            
            if valid_extension:
                new_branch = RRTBranch(nearest_branch, new_config)
                tree.append(new_branch)
                
                if n % 100 == 0:
                    self.get_logger().info(f"Tree size: {len(tree)}, Distance to goal: {distance_to_goal:.3f}")
                
                if distance_to_goal < goal_threshold:
                    goal_branch = RRTBranch(new_branch, joints_goal)
                    tree.append(goal_branch)
                    self.get_logger().info(f"RRT found path in {n} iterations")
                    break
        else:
            self.get_logger().warn("RRT failed to find path within iteration limit")
            return

        path = self.extract_path(goal_branch)
        optimized_path = self.short_cut_path(path)
        resampled_path = self.resample_path(optimized_path)

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        dt = 0.1 
        for i, waypoint in enumerate(resampled_path):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = rclpy.duration.Duration(seconds=i*dt).to_msg()
            trajectory.points.append(point)
        
        self.pub.publish(trajectory)
        self.get_logger().info(f"Published trajectory with {len(resampled_path)} waypoints")

    def segment_valid(self, node_goal, nearest_node, resolution=0.1):
        """Check validity of a segment between two configurations with adaptive resolution.
        1. Segment vector and distance:
            - compute the direction vector from nearest_node to node_goal and find EUCLIDEAN distance between them
            - if dist -> 0 then nodes are identical
        2. adaptive step calc
            - find the num of interpolation steps based on node_dist and specified resolution.
            - find step size by dividing the diirection vector by num_steps
        3. collision check
            - iteratively add step size to create intermediate configs along segment
            - each config is check for validity
            - if collision detected, segment = invalid!
        4. if all configs valid, then true!

        Args:
            node_goal: Target configuration
            nearest_node: Starting configuration
            resolution: Maximum step size in radians
            
        Returns:
            tuple: (valid_extension, last_valid_config)
        """
        node_direction = numpy.array(node_goal) - numpy.array(nearest_node)
        node_distance = numpy.linalg.norm(node_direction)
        
        if node_distance < 1e-6:  # Nodes are practically identical
            return True, nearest_node
        
        # Adaptive number of substeps based on distance
        num_steps = max(int(node_distance / resolution), 5)
        step = node_direction / num_steps
        
        current_config = numpy.array(nearest_node)
        
        for i in range(1, num_steps + 1):
            next_config = current_config + step
            next_config = numpy.mod(next_config + numpy.pi, 2 * numpy.pi) - numpy.pi
            
            if not self.is_state_valid(next_config):
                if i == 1: 
                    return False, nearest_node
                # Return last valid configuration
                return True, current_config.tolist()
            
            current_config = next_config
        
        return True, current_config.tolist()

    def short_cut_path(self, path, max_attempts=100):
        """Optimize path using bidirectional shortcutting with adaptive step sizes.
        1. Exit early for short paths
            - if path has <=2 points then return siply
            - else select 2 points randomly on path and if they are consequeitve then one of the attempts is skipped
            - if segment between 2 points is valid, then path between 2 nodes is removed and direct segment is path
        Args:
            path: List of configurations
            max_attempts: Maximum number of shortcut attempts
            
        Returns:
            list: Optimized path
        """
        if len(path) <= 2:
            return path
        
        optimized_path = path.copy()
        successful_cuts = 0
        
        for attempt in range(max_attempts):
            # Randomly select two points on the path
            idx1, idx2 = sorted(random.sample(range(len(optimized_path)), 2))
            if idx2 - idx1 <= 1:
                continue
                
            validity, _ = self.segment_valid(optimized_path[idx2], optimized_path[idx1])
            
            if validity:
                optimized_path[idx1+1:idx2] = []
                successful_cuts += 1
                
                if successful_cuts % 10 == 0:
                    self.get_logger().info(f"Path shortened from {len(path)} to {len(optimized_path)} points")
                
                if attempt > 20 and successful_cuts == 0:
                    break
        
        return optimized_path

    def resample_path(self, path, min_distance=0.1, max_distance=0.5):
        """Resample path with adaptive resolution based on curvature.
        1. If the path has fewer than two waypoints, it is returned without modification
        2. for each segment between consequetive waypoints
            - find the segment length
            - determine the resampling distance based on segment curvature -> high curvature = dense sampling
        3. For each segment, calculate the required number of samples and interpolates intermediate points.
        Args:
            path: List of configurations
            min_distance: Minimum distance between samples
            max_distance: Maximum distance between samples
            
        Returns:
            list: Resampled path
        """
        if len(path) < 2:
            return path
            
        resampled_path = [path[0]]
        
        for i in range(1, len(path)):
            segment = numpy.array(path[i]) - numpy.array(path[i-1])
            segment_length = numpy.linalg.norm(segment)
            
            if i > 1:
                prev_segment = numpy.array(path[i-1]) - numpy.array(path[i-2])
                angle = numpy.arccos(numpy.clip(
                    numpy.dot(segment, prev_segment) / 
                    (numpy.linalg.norm(segment) * numpy.linalg.norm(prev_segment)),
                    -1.0, 1.0))
                distance = min_distance + (max_distance - min_distance) * (1 - angle/numpy.pi)
            else:
                distance = (min_distance + max_distance) / 2
                
            num_samples = max(int(segment_length / distance), 1)
            
            for j in range(1, num_samples):
                alpha = j / num_samples
                intermediate_point = numpy.array(path[i-1]) + alpha * segment
                resampled_path.append(intermediate_point.tolist())
                
            resampled_path.append(path[i])
        
        return resampled_path

    def extract_path(self, goal_branch):
        """Extract the path from the RRT tree by following parent pointers from goal to start.
        1. Starting from goal_branch, trace back through each node’s parent until reaching the initial node
        2. add each node's configuration (q) to the path list in reverse order
        Args:
            goal_branch: The RRTBranch node at the goal configuration
            
        Returns:
            list: Path from start to goal as a list of configurations
        """
        path = []
        current = goal_branch
        
        # Traverse from goal to start by following parent pointers
        while current is not None:
            path.insert(0, current.q)  # Insert at beginning to maintain correct order
            current = current.parent
            
        return path

    def find_closest_point_in_tree(self, tree, r):
        r = numpy.array(r)  # Ensure numpy array
        shortest_distance = numpy.linalg.norm(r - numpy.array(tree[0].q))
        closest_point = tree[0]
        for i in range(1, len(tree)):
            distance = numpy.linalg.norm(r - numpy.array(tree[i].q))
            if distance < shortest_distance:
                shortest_distance = distance
                closest_point = tree[i]
        return closest_point

    
    def IK(self, T_goal):
        """ This function will perform IK for a given transform T of the 
        end-effector. It .

        Returns:
            q: returns a list q[] of values, which are the result 
            positions for the joints of the robot arm, ordered from proximal 
            to distal. If no IK solution is found, it returns an empy list
        """

        req = moveit_msgs.srv.GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = 'base'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        
        self.get_logger().info('Sending IK request...')
        res = self.ik_service.call(req)
        self.get_logger().info('IK request returned')
        
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        for i in range(0,len(q)):
            while (q[i] < -math.pi): q[i] = q[i] + 2 * math.pi
            while (q[i] > math.pi): q[i] = q[i] - 2 * math.pi
        return q

    
    def get_joint_info(self):
        '''This is a function which will collect information about the robot which
        has been loaded from the parameter server. It will populate the variables
        self.num_joints (the number of joints), self.joint_names and
        self.joint_axes (the axes around which the joints rotate)
        '''
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
        self.get_logger().info('Num joints: %d' % (self.num_joints))


    
    def is_state_valid(self, q):
        """ This function checks if a set of joint angles q[] creates a valid state,
        or one that is free of collisions. The values in q[] are assumed to be values
        for the joints of the UR5 arm, ordered from proximal to distal.

        Returns:
            bool: true if state is valid, false otherwise
        """
        req = moveit_msgs.srv.GetStateValidity.Request()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = list(q)
        req.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()

        res = self.state_valid_service.call(req)

        return res.valid



class RRTBranch(object):
    '''This is a class which you can use to keep track of your tree branches.
    It is easiest to do this by appending instances of this class to a list 
    (your 'tree'). The class has a parent field and a joint position field (q). 
    
    You can initialize a new branch like this:
        RRTBranch(parent, q)
    Feel free to keep track of your branches in whatever way you want - this
    is just one of many options available to you.
    '''
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


def main(args=None):
    rclpy.init(args=args)
    ma = MoveArm()
    ma.get_logger().info("Move arm initialization done")
    executor = MultiThreadedExecutor()
    executor.add_node(ma)
    executor.spin()
    ma.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()