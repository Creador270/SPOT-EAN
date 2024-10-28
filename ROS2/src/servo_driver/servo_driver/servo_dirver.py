import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_driver.servo_controller_fix import Controllers

class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_dirver_node')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )

        self.controller = Controllers()

    def joint_states_callback(self, msg):
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort

        self.controller.servoRotate(self.position)



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