import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped


HEARTBEAT_PERIOD = 0.1  # seconds
WAIT_TIME = 10.0        # seconds

TORQUE_DOOR_OPEN = 100.0    # N*m
TORQUE_DOOR_CLOSED = -100.0 # N*m


class Lab2(Node):
    def __init__(self):
        super().__init__('lab2')

        self.time_init = None

        self.door_torque_pub = self.create_publisher(
            Float64,
            'door_torque',
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10,
        )

        self.timer = self.create_timer(
            HEARTBEAT_PERIOD,
            self.heartbeat,
        )

        self.get_logger().info('Lab 2 initialized.')

    def heartbeat(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            self.get_logger().warn(
                'Waiting for a nonzero ROS clock time...',
                throttle_duration_sec=2.0,
            )
            return

        # Establish the timer only after the current clock is usable.
        if self.time_init is None:
            self.time_init = now
            self.get_logger().info(
                f'Elapsed timer started at {now.nanoseconds / 1e9:.3f} s.'
            )
            return

        elapsed = (now - self.time_init).nanoseconds / 1e9

        torque_msg = Float64()
        cmd_msg = TwistStamped()

        cmd_msg.header.stamp = now.to_msg()

        if elapsed < WAIT_TIME:
            torque_msg.data = TORQUE_DOOR_OPEN
            cmd_msg.twist.linear.x = 1.0
        else:
            torque_msg.data = TORQUE_DOOR_CLOSED
            cmd_msg.twist.linear.x = 0.0

        self.door_torque_pub.publish(torque_msg)
        self.cmd_vel_pub.publish(cmd_msg)

        self.get_logger().info(f'heartbeat: {elapsed:.2f} s')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    lab2 = Lab2()
    lab2.spin()
    lab2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
