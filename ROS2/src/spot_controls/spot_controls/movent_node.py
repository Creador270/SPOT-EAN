import rclpy
import copy
import math
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait

class ControlerSpot(Node):
    def __init__(self):
        super().__init__('spot_controls')

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

        self.declare_parameter('fr', 40.0)#Hz
        frecuency = self.get_parameter('fr').get_parameter_value().double_value

        self.lin_vel_x = 0.0
        self.lin_vel_y = 0.0
        self.ang_vel_z = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.height_z = 0.0
        self.stop = 1
        self.reset = 0
        
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
        # Gait Parameters
        self.BaseStepVelocity = 0.1
        self.StepVelocity = self.BaseStepVelocity
        # Stock, use Bumpers to change
        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = self.BaseSwingPeriod
        # Stock, use arrow pads to change
        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth = 0.005
        self.ClearanceHeight = self.BaseClearanceHeight
        self.PenetrationDepth = self.BasePenetrationDepth

        # Contacts: FL, FR, BL, BR
        self.contacts = [0, 0, 0, 0]

        # IMU: R, P, Ax, Ay, Az, Gx, Gy, Gz, Qx, Qy, Qz, Qw
        self.imu = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.load_spot()

        # Create timer for routine execution
        self.create_timer(1/frecuency, self.move) 
        
        self.subscription = self.create_subscription(Imu, 'imu/data', self.imuMagCallback, 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joystickCallback, 10)
        self.time = self.get_clock().now().seconds_nanoseconds()[0]
        # print("#########################################################")
        # print(self.time)
        # print("#########################################################")
        # time.sleep(10)

    def load_spot(self):
        # Load Environment
        #self.env = spotBezierEnv(render=False,
        #                         on_rack=False,
        #                         height_field=False,
        #                         draw_foot_path=False)

        #self.env.reset()

        #seed = 0
        ## Set seeds
        #self.env.seed(seed)
        #np.random.seed(seed)

        #state_dim = self.env.observation_space.shape[0]
        #print("STATE DIM: {}".format(state_dim))
        #action_dim = self.env.action_space.shape[0]
        #print("ACTION DIM: {}".format(action_dim))
        #max_action = float(self.env.action_space.high[0])
        #print("RECORDED MAX ACTION: {}".format(max_action))

        #self.state = self.env.reset()

        #self.dt = self.env._time_step

        # Load Spot Model
        self.spot = SpotModel()

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait()#dt=self.env._time_step)

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

    def joystickCallback(self, msg): 
        #print(self.height_z)
        try:
            self.lin_vel_x = (msg.axes[1]**2) * 0.1
            if msg.axes[1] < 0:
                self.lin_vel_x = self.lin_vel_x * -1
            self.lin_vel_y = (msg.axes[0]**2) * 0.1
            if msg.axes[0] < 0:
                self.lin_vel_y = self.lin_vel_y * -1

            if msg.buttons[4]:
                self.y = np.radians((msg.axes[4] ** 2)/ 0.05)
                if msg.axes[4] < 0:
                    self.y = self.y * -1
                self.x = np.radians((msg.axes[3] ** 2)/ 0.05)
                if msg.axes[3] < 0:
                    self.x = self.x * -1
            else:
                self.ang_vel_z = msg.axes[3]
            #self.z = msg.axes[5]
            self.height_z = (msg.axes[7] * 0.001) + self.height_z
            self.stop = msg.buttons[1]
            self.reset = msg.buttons[3]
        except KeyboardInterrupt:
            log.error("can't get data from joystick")

    def imuMagCallback(self, msg):
        self.imu[-3:] = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def publish_joints(self, joints_state):
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        if len(joints_state) == len(self.joint_names):
            #print(f'thetas: {joints_state}, {len(joints_state)}')
            msg.name = self.joint_names
            msg.position = joints_state.tolist()
        else:
            self.get_logger().error('Invalid number of joints: {}'.format(len(joints_state)))
            self.get_logger().error('Info Error: {}'.format(joints_state))
        msg.velocity = []
        msg.effort = []
        self.joint_publisher.publish(msg)

    def move(self):
        """ Turn joystick inputs into commands
        """

        if self.stop == 0:
            self.StepVelocity = self.BaseStepVelocity
            # self.SwingPeriod = np.clip(
            #     self.BaseSwingPeriod +
            #     (-self.faster + -self.action.slower) * SV_SCALE,
            #     0.1, 0.3)
            
            StepLength = self.lin_vel_x + abs(
                self.lin_vel_y * 0.66)
            StepLength = np.clip(StepLength, -1.0, 1.0)
            LateralFr= self.lin_vel_y * np.pi / 2
            YawRate = self.ang_vel_z
            # x offset
            pos = np.array(
                [0.0, 0.0, self.height_z])
            orn = np.array([self.x, self.y, 0.0])
            orn[:2] -= self.quaternion_to_euler(self.imu[-3:])[:2]
            # print("#########################################################")
            # print(pos)
            # print("#########################################################")
            # time.sleep(10)
        else:
            StepLength = 0.0
            LateralFr= 0.0
            YawRate = 0.0
            # RESET
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            # self.height_z = 0.0
            self.x = 0.0
            self.y = 0.0
            pos = np.array([0.0, 0.0, 0.0])
            orn = np.array([0.0, 0.0, 0.0])
            orn[:2] -= self.quaternion_to_euler(self.imu[-3:])[:2]
            time.sleep(0.5)

        # TODO: integrate into controller
        # self.ClearanceHeight += self.jb.updown * CHPD_SCALE
        # self.PenetrationDepth += self.jb.leftright * CHPD_SCALE

        # Manual Reset
        if self.reset == 1:
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            # self.env.reset()


        # print("SL: {} \tSV: {} \nLAT: {} \tYAW: {}".format(
        #     StepLength, self.StepVelocity, LateralFr YawRate))
        # print("BASE VEL: {}".format(self.BaseStepVelocity))
        # print("---------------------------------------")

        contacts = [0, 0, 0, 0]

        # Time
        dt = self.get_clock().now().seconds_nanoseconds()[0] - self.time
        # print("dt: {}".format(dt))
        self.time = self.get_clock().now().seconds_nanoseconds()[0]

        # Update Step Period
        self.bzg.Tswing = self.SwingPeriod

        self.T_bf = self.bzg.GenerateTrajectory(StepLength, LateralFr,
                                                YawRate, self.StepVelocity,
                                                self.T_bf0, self.T_bf,
                                                self.ClearanceHeight,
                                                self.PenetrationDepth,
                                                contacts)

        # for i, (key, Tbf_in) in enumerate(self.T_bf.items()):
        #     print("{}: \t Angle: {}".format(key, Tbf_in[:3, 3]))
        # print("-------------------------")

        joint_angles = self.spot.IK(orn, pos, self.T_bf)
        joint_angles = joint_angles.reshape(-1) * -1
        #self.env.pass_joint_angles(joint_angles)
        self.publish_joints(joint_angles)

        # Get External Observations
        # TODO
        # self.env.spot.GetExternalObservations(bzg, bz_step)
        # Step
        #action= self.env.action_space.sample()
        #action[:] = 0.0
        #self.state, reward, done, _ = self.env.step(action)

def main(args=None):
    print('Hi from spot_controls.')
    rclpy.init(args=args)
    try:
        node = ControlerSpot()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
