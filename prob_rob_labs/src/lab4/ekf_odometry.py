import math

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState


CMD_VEL_TOPIC = 'cmd_vel'
JOINT_STATES_TOPIC = 'joint_states'
IMU_TOPIC = 'imu'
ODOM_TOPIC = 'ekf_odom'
LEFT_WHEEL_JOINT = 'wheel_left_joint'
RIGHT_WHEEL_JOINT = 'wheel_right_joint'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.1435
CMD_TIMEOUT = 1.0
SYNC_SLOP = 0.1
ENCODER_VARIANCE = 0.1
INITIAL_GYRO_VARIANCE = 0.01
INITIAL_STATE_VARIANCE = np.diag([0.1, 0.1, 0.1, 0.5, 0.5])
PROCESS_NOISE = np.diag([0.2, 1.0])

ROBOT_RADIUS = WHEEL_SEPARATION / 2.0
Hx = np.array([
    [0.0, 0.0, 0.0, 1.0 / WHEEL_RADIUS, ROBOT_RADIUS / WHEEL_RADIUS],
    [0.0, 0.0, 0.0, 1.0 / WHEEL_RADIUS, -ROBOT_RADIUS / WHEEL_RADIUS],
    [0.0, 0.0, 0.0, 0.0, 1.0],
])

class EkfOdometry(Node):
    def __init__(self):
        super().__init__('ekf_odometry')

        self.linear_gain = float(self.declare_parameter('linear_gain', 1.0).value)
        self.angular_gain = float(self.declare_parameter('angular_gain', 1.0).value)
        self.linear_tau = float(self.declare_parameter('linear_tau', 0.21).value)
        self.angular_tau = float(self.declare_parameter('angular_tau', 0.05).value)

        if self.linear_tau <= 0.0 or self.angular_tau <= 0.0:
            raise ValueError('linear_tau and angular_tau must be greater than zero')

        self._state = np.zeros(5, dtype=float)
        self._covariance = INITIAL_STATE_VARIANCE.copy()
        self._last_stamp_ns = None
        self._last_command = np.zeros(2, dtype=float)
        self._last_command_time_ns = None
        self._warned_missing_joints = False

        self._odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.create_subscription(TwistStamped, CMD_VEL_TOPIC, self._command_callback, 10)

        self._joint_subscriber = message_filters.Subscriber(
            self, JointState, JOINT_STATES_TOPIC,
            qos_profile=qos_profile_sensor_data)
        self._imu_subscriber = message_filters.Subscriber(
            self, Imu, IMU_TOPIC,
            qos_profile=qos_profile_sensor_data)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._joint_subscriber, self._imu_subscriber],
            queue_size=10,
            slop=SYNC_SLOP)
        self._synchronizer.registerCallback(self._sensor_callback)

        self.get_logger().info(
            f'Fusing {JOINT_STATES_TOPIC} and {IMU_TOPIC}; '
            f'publishing EKF odometry on {ODOM_TOPIC}')

    def _command_callback(self, msg):
        self._last_command[:] = [msg.twist.linear.x, msg.twist.angular.z]
        self._last_command_time_ns = self.get_clock().now().nanoseconds

    def _active_command(self, now_ns):
        if self._last_command_time_ns is None:
            return np.zeros(2, dtype=float)
        age = (now_ns - self._last_command_time_ns) * 1.0e-9
        if age < 0.0 or age > CMD_TIMEOUT:
            return np.zeros(2, dtype=float)
        return self._last_command.copy()

    def _sensor_callback(self, joints, imu):
        joint_index = {name: index for index, name in enumerate(joints.name)}
        required = (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT)
        if any(name not in joint_index for name in required) or \
                len(joints.velocity) != len(joints.name):
            if not self._warned_missing_joints:
                self.get_logger().error(
                    'JointState must contain velocity values for '
                    f'{LEFT_WHEEL_JOINT} and {RIGHT_WHEEL_JOINT}')
                self._warned_missing_joints = True
            return

        left_rate = joints.velocity[joint_index[LEFT_WHEEL_JOINT]]
        right_rate = joints.velocity[joint_index[RIGHT_WHEEL_JOINT]]
        gyro_rate = imu.angular_velocity.z
        if not all(math.isfinite(value) for value in (left_rate, right_rate, gyro_rate)):
            self.get_logger().warning('Ignoring non-finite wheel or gyro measurement')
            return

        left_stamp_ns = self._stamp_to_ns(joints.header.stamp)
        imu_stamp_ns = self._stamp_to_ns(imu.header.stamp)
        stamp_ns = (left_stamp_ns + imu_stamp_ns) // 2
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self.get_logger().warning('Ignoring out-of-order synchronized sensor data')
            return

        measured_velocity = WHEEL_RADIUS * (right_rate + left_rate) / 2.0
        measured_yaw_rate = (
            WHEEL_RADIUS * (right_rate - left_rate) / WHEEL_SEPARATION
        )
        measurement = np.array([right_rate, left_rate, gyro_rate], dtype=float)

        if self._last_stamp_ns is not None:
            dt = (stamp_ns - self._last_stamp_ns) * 1.0e-9
            self._predict(dt, self._active_command(self.get_clock().now().nanoseconds))
        else:
            self._state[3] = measured_velocity
            self._state[4] = measured_yaw_rate

        self._update(measurement, imu)
        self._last_stamp_ns = stamp_ns
        self._publish_odometry(stamp_ns)

    @staticmethod
    def _stamp_to_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _predict(self, dt, command):
        theta, x, y, velocity, yaw_rate = self._state
        a_v = 0.1 ** (dt / self.linear_tau)
        a_w = 0.1 ** (dt / self.angular_tau)

        self._state = np.array([
            theta + yaw_rate * dt,
            x + velocity * math.cos(theta) * dt,
            y + velocity * math.sin(theta) * dt,
            a_v * velocity + self.linear_gain * (1.0 - a_v) * command[0],
            a_w * yaw_rate + self.angular_gain * (1.0 - a_w) * command[1],
        ])

        Gx = np.array([
            [1.0, 0.0, 0.0, 0.0, dt],
            [-velocity * math.sin(theta) * dt, 1.0, 0.0, math.cos(theta) * dt, 0.0],
            [velocity * math.cos(theta) * dt, 0.0, 1.0, math.sin(theta) * dt, 0.0],
            [0.0, 0.0, 0.0, a_v, 0.0],
            [0.0, 0.0, 0.0, 0.0, a_w],
        ])
        Gu = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [1 - a_v, 0],
            [0, 1 - a_w],
        ])
        self._covariance = (
            Gx @ self._covariance @ Gx.T + Gu @ PROCESS_NOISE @ Gu.T
        )

    def _update(self, measurement, imu):
        gyro_variance = INITIAL_GYRO_VARIANCE
        imu_variance = imu.angular_velocity_covariance[8]
        if math.isfinite(imu_variance) and imu_variance > 0.0:
            gyro_variance = imu_variance

        measurement_covariance = np.diag([
            ENCODER_VARIANCE,
            ENCODER_VARIANCE,
            gyro_variance,
        ])

        expected_measurement_mean = Hx @ self._state # b.c. h(x) is linear

        S = Hx @ self._covariance @ Hx.T + measurement_covariance
        C = self._covariance @ Hx.T
        K_gain = np.linalg.solve(S.T, C.T).T
        self._state += K_gain @ (measurement - expected_measurement_mean)
        self._covariance = (np.eye(5) - K_gain @ Hx) @ self._covariance

    def _publish_odometry(self, stamp_ns):
        msg = Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000)
        msg.header.frame_id = ODOM_FRAME
        msg.child_frame_id = BASE_FRAME

        theta, x, y, velocity, yaw_rate = self._state
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.twist.twist.linear.x = velocity
        msg.twist.twist.angular.z = yaw_rate

        pose_indices = [1, 2, 0]
        pose_slots = [0, 1, 5]
        for row, pose_row in enumerate(pose_slots):
            for col, pose_col in enumerate(pose_slots):
                msg.pose.covariance[pose_row * 6 + pose_col] = float(
                    self._covariance[pose_indices[row], pose_indices[col]])

        twist_indices = [3, 4]
        twist_slots = [0, 5]
        for row, twist_row in enumerate(twist_slots):
            for col, twist_col in enumerate(twist_slots):
                msg.twist.covariance[twist_row * 6 + twist_col] = float(
                    self._covariance[twist_indices[row], twist_indices[col]])

        self._odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EkfOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
