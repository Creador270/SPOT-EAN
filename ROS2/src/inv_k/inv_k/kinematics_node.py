import rclpy
import math
import time
import numpy as np
#import jax.numpy as jnp
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from inv_k.kinematics import Kinematic

class kinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')
        # self.subscription = self.create_subscription(Pose, 'Pose', self.stateCallback, 10)

        # Configure QoS profile for better real-time performance
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize publisher with QoS profile
        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )

        # self.subscription = self.create_subscription(Imu, 'imu/data', self.imuMagCallback, 10)
        # self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.invk_model = Kinematic()

        # self.joint_state_msg = JointState()
        self.joint_names = [
            'joint_front_left_shoulder',  # front_left UP
            'joint_front_left_leg',  # front_left MID
            'joint_front_left_foot',   # front_left DOWN
            'joint_front_right_shoulder',   # front_right UP
            'joint_front_right_leg',  # front_right MID
            'joint_front_right_foot',  # front_right DOWN
            'joint_rear_left_shoulder',  # back_left UP
            'joint_rear_left_leg',  # back_left MID
            'joint_rear_left_foot', # back_left DOWN
            'joint_rear_right_shoulder',  # back_right UP
            'joint_rear_right_leg', # back_right MID
            'joint_rear_right_foot'   # back_right DOWN
        ]
        # self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)
        # self.joint_state_msg.velocity = []
        # self.joint_state_msg.effort = []
        
        # Increment
        self.height_increment = 30

        # Inicial State
        self.legIndex = [0, 3]
        self.legState = np.array([1, 1, 1, 1])
        self.legEndpoints=np.array([[65,-60,87.5,1],
                                    [65,-60,-87.5,1],
                                    [-155,-60,87.5,1],
                                    [-155,-60,-87.5,1]])
        
        # Create timer for routine execution
        self.create_timer(0.1, self.timer_callback)

        time.sleep(2) # Wait to the user can verify the robot

        self.get_logger().info('Spot Joint Publisher Node initialized')
        # self.timer = self.create_timer(0.1, self.test_actions)  # Publica cada 0.1 segundos

        # self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
        # self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
        # self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        # self.publisher_.publish(self.joint_state_msg)

        # time.sleep(3)
        
        # for i in np.arange(-60, -175, -10):
        #     self.legEndpoints[:, 1] = i
        #     self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
        #     self.joint_state_msg.position = list(self.invk_model.thetas.flatten())
        #     self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        #     self.publisher_.publish(self.joint_state_msg)

        # time.sleep(2)
    def publish_joints(self, joints_state):
        print(f'thetas: {joints_state}')
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        if len (joints_state) == len(self.joint_names):
            msg.name = self.joint_names
            msg.position = joints_state.tolist()
        else:
            self.get_logger().error('Invalid number of joints')
        msg.velocity = []
        msg.effort = []
        self.joint_publisher.publish(msg)

    def test_actions(self):
        """
        Calculate the inverse kinematics for a given end
        effector position using theinverse kinematics model.
        """
        try:
            # Calculate the inverse kinematics
            joint_angles = self.invk_model.calcIK(
                self.legEndpoints,  #position of each leg
                (0,0,0),            #orientation of the robot
                (0,0,0)             #position of the center poin of the robot
                )
            return joint_angles.flatten()
        
        except Exception as e:
            self.get_logger().error(f'Error calculating joint positions: {str(e)}')
            return None
        
        # for i in range(self.legEndpoints.shape[0]):
        #     for j in range(-150, -60, 10):
        #         self.legEndpoints[i][1] = j
        #         self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
        #         self.publish_joints()
        #     for j in range(-60, -150, -10):
        #         self.legEndpoints[i][1] = j
        #         self.invk_model.drawRobot(self.legEndpoints, (0,0,0), (0,0,0))
        #         self.publish_joints()
    
    def timer_callback(self):
        """ 
        This function is called every 0.1 seconds to execute the inverse kinematics.
        """
        joints_positions = self.test_actions()

        if joints_positions is not None:

            self.publish_joints(joints_positions) #Publish the joint positions

            #Update the joint_positions of the robot
            if all(self.legState == 1):
                self.legEndpoints[:, 1] -= 10
                if self.legEndpoints[:, 1].min() <= -175:
                    self.legState[:] = 0
                
            else:
                self.stabitily_check()  
        # self.joint_state_msg.position = self.test_actions()
        # self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        # self.publisher_.publish(self.joint_state_msg)

    def stabitily_check(self):
        two_leg_index_map = {(0, 3): [1, 2], (1, 2): [0, 3]}
        if all(self.legState[self.legIndex] == 0):
            self.legEndpoints[self.legIndex, 1] += self.height_increment
            if self.legEndpoints[self.legIndex, 1].max() >= -60:
                self.legState[self.legIndex] = 1
        else:
            self.legEndpoints[self.legIndex, 1] -= self.height_increment
            if self.legEndpoints[self.legIndex, 1].min() <= -150:
                self.legState[self.legIndex] = 0
                self.legIndex = two_leg_index_map.get(tuple(self.legIndex))

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

    def rotations(self):
        self.invk_model.drawRobot(self.legEndpoints, (np.radians(20),0,0), (0,0,0))
        self.publish_joints()

        for yaw in range(0, 20):
            self.invk_model.drawRobot(self.legEndpoints, (np.radians(yaw),0,0), (0,0,0))
            self.publish_joints()
        for yaw in range(20, 0):
            self.invk_model.drawRobot(self.legEndpoints, (np.radians(yaw),0,0), (0,0,0))
            self.publish_joints()
        for yaw in range(-20, 0):
            self.invk_model.drawRobot(self.legEndpoints, (np.radians(yaw),0,0), (0,0,0))
            self.publish_joints()
            
        for pitch in range(0, 20):
            self.invk_model.drawRobot(self.legEndpoints, (0,np.radians(pitch),0), (0,0,0))
            self.publish_joints()
        for pitch in range(20, -20):
            self.invk_model.drawRobot(self.legEndpoints, (0,np.radians(pitch),0), (0,0,0))
            self.publish_joints()

def main(args=None):
  rclpy.init(args=args)
  try:
      node = kinematicsNode()
      rclpy.spin(node)
  except KeyboardInterrupt:
      pass
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()
