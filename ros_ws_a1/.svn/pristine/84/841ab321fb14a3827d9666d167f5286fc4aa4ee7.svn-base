import numpy

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import transforms3d

class Autograder(Node):

    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        timer_period = 2.0  
        self.timer = self.create_timer(timer_period, self.grade)

        
    def check_transform(self, aff, t):
        trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rot = [t.transform.rotation.w, t.transform.rotation.x,  t.transform.rotation.y, t.transform.rotation.z]
        T,R,Z,S = transforms3d.affines.decompose(aff)
        if not (abs(trans - T) < 10e-4).all():
            return False        
        if not transforms3d.quaternions.nearly_equivalent(rot, transforms3d.quaternions.mat2quat(R)):
            return False
        return True


    def grade(self):

      b_T_o = numpy.dot(transforms3d.affines.compose((0.0,0.0,0), transforms3d.euler.euler2mat(0.64, 0.64, 0.0,'rxyz'), (1,1,1)), 
                        transforms3d.affines.compose((1.5,0.8,0), numpy.eye(3,3), (1,1,1)) )
      b_T_r = numpy.dot(transforms3d.axangles.axangle2aff([0.0, 1.0, 0.0], 1.5),
                        transforms3d.affines.compose((0.0,0.0,-2.0), numpy.eye(3,3), (1,1,1)) )

      r_T_c_translation = [0.3,0.0,0.3]
      r_T_c_no_rotation = transforms3d.affines.compose(r_T_c_translation, numpy.eye(3,3), (1,1,1))
      b_T_c_no_rotation = numpy.dot(b_T_r, r_T_c_no_rotation)
      c_T_o_no_rotation = numpy.dot(numpy.linalg.inv(b_T_c_no_rotation), b_T_o)
      current_x_axis = [1,0,0]
      T,R,Z,S = transforms3d.affines.decompose(c_T_o_no_rotation)
      desired_x_axis = T
      rotation_axis_btwn_x_axes = numpy.cross(current_x_axis, desired_x_axis)
      rotation_angle_btwn_x_axes = numpy.arccos(numpy.dot(current_x_axis, desired_x_axis)/(numpy.linalg.norm(current_x_axis) * numpy.linalg.norm(desired_x_axis)))

      r_T_c_rotation = transforms3d.axangles.axangle2aff(rotation_axis_btwn_x_axes, rotation_angle_btwn_x_axes)
    
      r_T_c = numpy.dot(r_T_c_no_rotation, r_T_c_rotation)

      b_T_c = numpy.dot(b_T_r, r_T_c)

      grade = 0

      print("First transform")
      try:
          t = self.tf_buffer.lookup_transform('base_frame', 'object_frame', rclpy.time.Time())
          correct = self.check_transform(b_T_o, t)        
          if correct:
              print("Object transform: 3")
              grade = grade + 3
          else:
              print("Object transform: 0")

      except TransformException:
          print("Base->Object: Did not receive valid transform")
          print("Object transform: 0")

      print("\n")
      print("2nd transform:")
      try:
          t = self.tf_buffer.lookup_transform('base_frame', 'robot_frame', rclpy.time.Time())
          b_T_r_student = transforms3d.affines.compose(
              (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
              transforms3d.quaternions.quat2mat([t.transform.rotation.w, t.transform.rotation.x,  t.transform.rotation.y, t.transform.rotation.z]),
              (1,1,1))
          correct = self.check_transform(b_T_r, t)        
          if correct:
              print("Robot transform: 3")
              grade = grade + 3
          else:
              print("Robot transform: 0")

      except TransformException:
          print("Base->Robot: Did not receive valid transform")
          print("Robot transform: 0")
          b_T_r_student = numpy.eye(4)
 
      print("\n")
      print("3rd Transform:")
      try:
          t = self.tf_buffer.lookup_transform('robot_frame', 'camera_frame', rclpy.time.Time())
          trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
          rot = [t.transform.rotation.w, t.transform.rotation.x,  t.transform.rotation.y, t.transform.rotation.z]
          T,R,Z,S = transforms3d.affines.decompose(r_T_c)
          correct = True
          if not (abs(trans - T) < 10e-4).all():
              correct = False   
          x_axis = b_T_c[:3, 0]
          r_T_c_student = transforms3d.affines.compose(
              (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
              transforms3d.quaternions.quat2mat([t.transform.rotation.w, t.transform.rotation.x,  t.transform.rotation.y, t.transform.rotation.z]),
              (1,1,1))
          b_T_c_student =  numpy.dot(b_T_r_student, r_T_c_student)
          x_axis_student = b_T_c_student[:3, 0]
          if not (abs(x_axis - x_axis_student) < 10e-4).all():
              correct = False      
          if correct:
              print("Camera transform: 4")
              grade = grade + 4
          else:
              print("Camera transform: 0")

      except TransformException:
          print("Robot->Camera: Did not receive valid transform")
          print("Camera transform: 0")

      print("\n")
      print("Grade: %d"%grade)
      self.get_logger().info(f'Grade: {grade}', once=True)


def main(args=None):
    rclpy.init(args=args)
    node = Autograder()
    print("Grading")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

