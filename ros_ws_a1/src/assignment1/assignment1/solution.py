# Implement your code here.

import transforms3d as t3d
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node

class Transformer_an3314(Node):
    def __init__(self):
        super().__init__('transformer_an3314')
        self.transform_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transforms) #10Hz and publishing all transforms

    def publish_transform(self, p_frame, c_frame, trans, rot):
        '''
        Function 1:
        - Publishes a transform
        - input: parent frame, child frame, rotated transform and quaternion  
        '''
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = p_frame #parent
        t.child_frame_id = c_frame #child
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z  = trans[2]
        t.transform.rotation.x = rot[1]
        t.transform.rotation.y = rot[2]
        t.transform.rotation.z = rot[3]
        t.transform.rotation.w = rot[0]

        self.transform_broadcaster.sendTransform(t) 
    
    def publish_transforms(self):
        self.publish_base_to_obj()
        self.publish_base_to_rob()
        self.publish_rob_to_cam()

    def publish_base_to_obj(self):
        '''
        base to object:
        - rotation by rpy = (0.64, 0.64, 0.64)
        - translation by (1.5, 0.8, 0)
        '''
        quat = t3d.euler.euler2quat(0.64, 0.64, 0.0, axes='rxyz')
        rotation_matrix = t3d.euler.euler2mat(0.64, 0.64, 0.0, axes='rxyz')
        translation = np.array([1.5, 0.8, 0.0])
        rotated_translation = rotation_matrix @ translation  # first rotate and then translate
        self.publish_transform('base_frame', 'object_frame', rotated_translation, quat)

    def publish_base_to_rob(self):
        '''
        base to robot:
        - rotation by rpy = (0, 1.5, 0)
        - translation by (0, 0, -2.0)
        '''
        quat = t3d.euler.euler2quat(0, 1.5, 0, axes='rxyz')
        rotation_matrix = t3d.euler.euler2mat(0, 1.5, 0, axes='rxyz')
        translation = np.array([0.0, 0.0, -2.0])
        rotated_translation = rotation_matrix @ translation 
        self.publish_transform('base_frame', 'robot_frame', rotated_translation, quat)

    def publish_rob_to_cam(self):
        '''
        robot to camera:
        - first the object is taken in the base's frame
        - then we get the robot's position in the base's frame
        - I then transform the object to be in the robot's frame
            --> using the rotated positions 
            --> Process is to find the vector from the robot's position to the object's position, 
                both expressed in the base frame.
                Then find the inverse (which is the same as the transpose of a rotation matrix) of rotation 
                matrix that gets the robots orientation in the base frame.
                Finally perform a matrix multiplication to rotate the position vector into the robots frame.
        - then I calculate the direction vector (then normalize it to get a UNIT vector)
            --> from the object's origin to the robot's origin
        - Then we randomly choose a z-axis (randomly chosen as 1, 1, 1)
        - y = z (cross) x
        - z = x (cross) y
        - finally we find the rotation matrix and the quaternion! and send it to get published!
        '''
        obj_rot_base = t3d.euler.euler2mat(0.64, 0.64, 0.0, axes='rxyz')
        obj_trans_base = np.array([1.5, 0.8, 0.0])
        obj_in_base = obj_rot_base @ obj_trans_base

        rob_rot_base = t3d.euler.euler2mat(0, 1.5, 0, axes='rxyz')
        rob_trans_base = np.array([0.0, 0.0, -2.0])
        rob_in_base = rob_rot_base @ rob_trans_base

        # Calculate object position in robot frame
        obj_in_rob = np.linalg.inv(rob_rot_base) @ (obj_in_base - rob_in_base)

        cam_in_rob = np.array([0.3, 0.0, 0.3])

        # Calculate direction vector (x-axis of camera frame)
        x_axis = obj_in_rob - cam_in_rob
        x_axis = x_axis / np.linalg.norm(x_axis)

        z_ref = np.array([1, 1, 1])

        y_axis = np.cross(z_ref, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        z_axis = np.cross(x_axis, y_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
        quat = t3d.quaternions.mat2quat(rotation_matrix)

        self.publish_transform('robot_frame', 'camera_frame', cam_in_rob, quat)

def main(args=None):
    rclpy.init(args=args)
    transform_publisher = Transformer_an3314()
    rclpy.spin(transform_publisher)
    transform_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()