import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

from inv_k.kinematics import Kinematic

class kinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')
        # self.subscription = self.create_subscription(Pose, 'Pose', self.stateCallback, 10)
        self.subscription = self.create_subscription(Imu, 'imu/data', self.imuMagCallback, 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

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

        self.declare_parameter('default_position', False)

        self.invk_model = Kinematic()

        default_position = self.get_parameter('default_position').get_parameter_value().bool_value

        self.joint_state_msg = JointState()
      
        self.joint_state_msg.name = list(self._servo_Articulations.keys()) # Nombres de las articulaciones
        self.joint_state_msg.position = [0.0] * 12
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []

        self.timer = self.create_timer(0.1, self.test_actions)  # Publica cada 0.1 segundos

        if default_position:
            self.legEndpoints=np.array([[60,-60,87.5,1],[60,-60,-87.5,1],[-100,-60,87.5,1],[-100,-60,-87.5,1]]), (0,0,0), (0,0,0)
            self.joint_state_msg.position = self.legEndpoints
            self.publisher_.publish(self.joint_state_msg)

    def euler_to_quaternion(roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw
    
    def imuMagCallback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"

        # Asignar la orientación del IMU al cuaternión de rotación
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        # Asignar la traslación (si es necesario, de lo contrario, dejar en cero)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

    def test_actions(self):
        legEndpoints = np.array([[60,-150,87.5,1],[60,-150,-87.5,1],[-100,-150,87.5,1],[-100,-150,-87.5,1]])
        
        for i in range(legEndpoints.shape[0]):
            for j in range(-150, -60, 10):
                legEndpoints[i][1] = j
                self.invk_model.drawRobot(legEndpoints, (0,0,0), (0,0,0))
                print(f'thetas: {(self.invk_model.thetas.flatten())}')
                self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
                self.publisher_.publish(self.joint_state_msg)
            for j in range(-60, -150, -10):
                legEndpoints[i][1] = j
                self.invk_model.drawRobot(legEndpoints, (0,0,0), (0,0,0))
                print(f'thetas: {(self.invk_model.thetas.flatten())}')
                self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
                self.publisher_.publish(self.joint_state_msg)

def main(args=None):
  rclpy.init(args=args)
  node = kinematicsNode()
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      pass
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()