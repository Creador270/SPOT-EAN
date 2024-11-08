import rclpy
import numpy as np
from servo_driver.kinematics import Kinematic 
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_driver.servo_controller import Controllers

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_dirver_node')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )

        self.declare_parameter('offset_joints', False)

        self.controller = Controllers()

        ofset_motors = self.get_parameter('offset_joints').get_parameter_value().bool_value

        if ofset_motors:
            self.invKin = Kinematic()
            self.invKin.drawRobot(np.array([[60,-60,87.5,1],[60,-60,-87.5,1],[-100,-60,87.5,1],[-100,-60,-87.5,1]]), (0,0,0), (0,0,0))
            self.controller.servoRotate(self.invKin.thetas)
        else:
            self.controller.servoRotate(np.zeros((4,3)))
            

    def joint_states_callback(self, msg):

        self._servo_Articulations = {
            'joint_front_left_shoulder': 0,  # front_left UP
            'joint_front_left_leg': 0,  # front_left MID
            'joint_front_left_foot': 0,   # front_left DOWN
            'joint_front_right_shoulder': 0,   # front_right UP
            'joint_front_right_leg': 0,  # front_right MID
            'joint_front_right_foot': 0,  # front_right DOWN
            'joint_rear_left_shoulder': 0,  # back_left UP
            'joint_rear_left_leg': 0,  # back_left MID
            'joint_rear_left_foot': 0, # back_left DOWN
            'joint_rear_right_shoulder': 0,  # back_right UP
            'joint_rear_right_leg': 0, # back_right MID
            'joint_rear_right_foot': 0   # back_right DOWN
        }

        self._servo_Articulations['joint_front_left_shoulder'] = msg.position[0]
        self._servo_Articulations['joint_front_left_leg'] = msg.position[1]
        self._servo_Articulations['joint_front_left_foot'] = msg.position[2]
        self._servo_Articulations['joint_front_right_shoulder'] = msg.position[3]
        self._servo_Articulations['joint_front_right_leg'] = msg.position[4]
        self._servo_Articulations['joint_front_right_foot'] = msg.position[5]
        self._servo_Articulations['joint_rear_left_shoulder'] = msg.position[6]
        self._servo_Articulations['joint_rear_left_leg'] = msg.position[7]
        self._servo_Articulations['joint_rear_left_foot'] = msg.position[8]
        self._servo_Articulations['joint_rear_right_shoulder'] = msg.position[9]
        self._servo_Articulations['joint_rear_right_leg'] = msg.position[10]
        self._servo_Articulations['joint_rear_right_foot'] = msg.position[11]

        self.front_left_leg = np.array(msg.position[:3])
        self.front_right_leg = np.array(msg.position[3:6])
        self.rear_left_leg = np.array(msg.position[6:9])
        self.rear_right_leg = np.array(msg.position[9:])

        if msg.name == list(self._servo_Articulations.keys()):
            self.radians = np.array([self.front_left_leg, self.front_right_leg, self.rear_left_leg, self.rear_right_leg])
            print(f'front_left_leg: {self.radians[0]}\n front_right_leg: {self.radians[1]}\n rear_left_leg: {self.radians[2]}\n rear_right_leg: {self.radians[3]}\n')
            self.controller.servoRotate(self.radians)
        else:
            print('joint states not in correct order')



def main(args=None):
  rclpy.init(args=args)
  node = ServoDriver()
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      pass
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()