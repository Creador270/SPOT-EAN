import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

class ImuBroadcaster(Node):
  def __init__(self):
      super().__init__('tf_broadcaster_imu_node')
      self.br = tf2_ros.TransformBroadcaster(self)
      self.subscription = self.create_subscription(
          Imu,
          'imu/data',
          self.enlace_imu,
          10
      )

  def enlace_imu(self, msg):
      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = "base_link"
      t.child_frame_id = "front_link"

      # Asignar la orientación del IMU al cuaternión de rotación
      t.transform.rotation.x = msg.orientation.x
      t.transform.rotation.y = msg.orientation.y
      t.transform.rotation.z = msg.orientation.z
      t.transform.rotation.w = msg.orientation.w

      # Asignar la traslación (si es necesario, de lo contrario, dejar en cero)
      t.transform.translation.x = 0.0
      t.transform.translation.y = 0.0
      t.transform.translation.z = 0.0

      self.br.sendTransform(t)

def main(args=None):
  rclpy.init(args=args)
  node = ImuBroadcaster()
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      pass
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()