import rclpy
import math
import time
import numpy as np
#import jax.numpy as jnp
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
        self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []

        self.timer = self.create_timer(0.1, self.test_actions)  # Publica cada 0.1 segundos

        self.legEndpoints=np.array([[60,-60,87.5,1],[60,-60,-87.5,1],[-100,-60,87.5,1],[-100,-60,-87.5,1]])
        
        time.sleep(1)

        self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
        self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)
        
        for i in np.arange(-60, -150, -5):
            self.invk_model.drawRobot(np.array([[60,i,87.5,1],[60,i,-87.5,1],[-100,i,87.5,1],[-100,i,-87.5,1]]), (0,0,0), (0,0,0))
            self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.joint_state_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw
    
    def quaternion_to_euler(self, x, y, z, w):
        ysqr = y * y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw
    
    def imuMagCallback(self, msg):
        # Optener la marca de tiempo del mensaje IMU
        self.dt = msg.header.stamp
        # Asignar la orientación del IMU al cuaternión de rotación
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        self.euler_angles = self.quaternion_to_euler(qx, qy, qz, qw)

    def publish_joints(self):
        #print(f'thetas: {(self.invk_model.thetas.flatten())}')
        self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)

    def rotations(self):
        for yaw in range(0, 20):
            self.invk_model.drawRobot(self.legEndpoints, (np.radians(yaw),0,0), (0,0,0))
            self.publish_joints()
            
        for pitch in range(0, 20):
            self.invk_model.drawRobot(self.legEndpoints, (0,np.radians(pitch),0), (0,0,0))
            self.publish_joints()
        for yaw in range(20, -20):
            self.invk_model.drawRobot(self.legEndpoints, (np.radians(yaw),0,0), (0,0,0))
            self.publish_joints()
        for pitch in range(20, -20):
            self.invk_model.drawRobot(self.legEndpoints, (0,np.radians(pitch),0), (0,0,0))
            self.publish_joints()

    def test_actions(self):        
        for i in range(self.legEndpoints.shape[0]):
            for j in range(-150, -60, 10):
                self.legEndpoints[i][1] = j
                self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
                self.publish_joints()
            for j in range(-60, -150, -10):
                self.legEndpoints[i][1] = j
                self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
                self.publish_joints()

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