import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu

from inv_k.inv_k.kinematics import Kinematic

class kinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')
        self.subscription = self.create_subscription(Pose, 'Pose', self.stateCallback, 10)
        self.subscription = self.create_subscription(Imu, 'imu_data', self.imuMagCallback, 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.declare_parameter('default_position', False)

        self.controller = Kinematic()

        default_position = self.get_parameter('default_position').get_parameter_value().bool_value

        if default_position:
            self.legEndpoints=np.array([[100,-100,87.5,1],[100,-100,-87.5,1],[-100,-100,87.5,1],[-100,-100,-87.5,1]])
            self.publisher_.publish(msg)
            # self.controller.servoRotate(self.controller._servo_offsets)

    def euler_to_quaternion(roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw
    