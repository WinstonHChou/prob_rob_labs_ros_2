import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import message_filters
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

import numpy as np
import tf_transformations as tft

SYNC_SLOP = 0.1

class EkfOdomMetric(Node):
    def __init__(self):
        super().__init__('ekf_odom_metric')

        self._ekf_odom_sub = message_filters.Subscriber(
            self, Odometry, 'ekf_odom', qos_profile=qos_profile_sensor_data)
        self._gt_pose_sub = message_filters.Subscriber(
            self, PoseStamped, 'gt_pose', qos_profile=qos_profile_sensor_data)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._ekf_odom_sub, self._gt_pose_sub],
            queue_size=10,
            slop=SYNC_SLOP)
        self._synchronizer.registerCallback(self._on_sync_data)

        self._error_metric_pub = self.create_publisher(Float64MultiArray, 'ekf_odom_error_metric', 10)

        self.get_logger().info('EkfOdom Metric initialized.')

    def _on_sync_data(
        self, ekf_odom_msg: Odometry, gt_pose_msg: PoseStamped):
        error_quat = tft.quaternion_multiply(
            tft.quaternion_conjugate([
                gt_pose_msg.pose.orientation.x,
                gt_pose_msg.pose.orientation.y,
                gt_pose_msg.pose.orientation.z,
                gt_pose_msg.pose.orientation.w
            ]),
            [
                ekf_odom_msg.pose.pose.orientation.x,
                ekf_odom_msg.pose.pose.orientation.y,
                ekf_odom_msg.pose.pose.orientation.z,
                ekf_odom_msg.pose.pose.orientation.w
            ]
        )
        error_euler = tft.euler_from_quaternion(error_quat)
        error_metric = Float64MultiArray()
        error_metric.data = [
            ekf_odom_msg.pose.pose.position.x - gt_pose_msg.pose.position.x,
            ekf_odom_msg.pose.pose.position.y - gt_pose_msg.pose.position.y,
            error_euler[2]
        ]
        self._error_metric_pub.publish(error_metric)


def main():
    rclpy.init()
    node = EkfOdomMetric()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
