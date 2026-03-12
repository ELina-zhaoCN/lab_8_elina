#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        odom_topic = self.get_parameter('odom_topic').value
        self._last_tf = None
        self._msg_count = 0
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._timer = self.create_timer(0.02, self._timer_cb)
        self.get_logger().info(f'Waiting for /odom on {odom_topic}...')
        # Publish static identity immediately so Nav2 can start
        self._publish_identity()

    def _publish_identity(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _odom_cb(self, msg: Odometry):
        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info('Received first /odom, switching to real TF')
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._last_tf = t

    def _timer_cb(self):
        now = self.get_clock().now()
        if self._last_tf is not None:
            self._last_tf.header.stamp = now.to_msg()
            self.tf_broadcaster.sendTransform(self._last_tf)
        else:
            # Keep publishing identity until real /odom arrives
            self._publish_identity()


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
