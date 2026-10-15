import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped


HEARTBEAT_PERIOD = 0.1  # seconds
WAIT_FOR_DRIVE = 10.0   # seconds

FEATURE_MEAN_X_THRESHOLD = 260 # px

TORQUE_DOOR_OPEN = 3.0    # N*m
TORQUE_DOOR_CLOSED = -3.0 # N*m

DEFAULT_SPEED = 0.5

STATE_OPEN = 'open'
STATE_DRIVE = 'drive'
STATE_STOP = 'stop'

class Lab3(Node):
    def __init__(self):
        super().__init__('lab3')

        self.declare_parameter('speed', DEFAULT_SPEED)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        if self.speed != DEFAULT_SPEED:
            self.get_logger().info(f'Speed set to {self.speed}')

        self.feature_mean_x_sub = self.create_subscription(Float64, 'feature_mean_x',
                                                           self.on_feature_mean_x, 10)
        self.feature_mean_x = None

        self.door_torque_pub = self.create_publisher(Float64, 'door_torque', 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.timer = self.create_timer(HEARTBEAT_PERIOD, self.heartbeat)
        self.state = None
        self.last_state_stamp = None

        self.get_logger().info('Lab 3 initialized.')

    def heartbeat(self):
        now = self.get_clock().now()

        # Initialize the state if it hasn't been set yet
        if self.last_state_stamp is None:
            self.state = STATE_OPEN
            self.last_state_stamp = now
            self.get_logger().info(f'state initialized to {self.state}.')
            return

        elapsed = (now - self.last_state_stamp).nanoseconds / 1e9
        # self.get_logger().info(f'Elapsed time since last state change: {elapsed:.2f} seconds')

        torque_msg = Float64()
        twist_msg = TwistStamped()

        # State machine for door and drive control
        if self.state == STATE_OPEN:
            if self.feature_mean_x is not None and \
               self.feature_mean_x < FEATURE_MEAN_X_THRESHOLD:
                self.state = STATE_DRIVE
                self.last_state_stamp = now
                self.get_logger().info(f'state changed to {self.state}.')
                return
            torque_msg.data = TORQUE_DOOR_OPEN
            self.door_torque_pub.publish(torque_msg)

        elif self.state == STATE_DRIVE:
            if elapsed >= WAIT_FOR_DRIVE:
                self.state = STATE_STOP
                self.last_state_stamp = now
                self.get_logger().info(f'state changed to {self.state}.')
                return
            twist_msg.header.stamp = now.to_msg()
            twist_msg.twist.linear.x = self.speed
            self.cmd_vel_pub.publish(twist_msg)

        elif self.state == STATE_STOP:
            torque_msg.data = TORQUE_DOOR_CLOSED
            twist_msg.header.stamp = now.to_msg()
            twist_msg.twist.linear.x = 0.0
            self.door_torque_pub.publish(torque_msg)
            self.cmd_vel_pub.publish(twist_msg)

    def on_feature_mean_x(self, msg):
        self.feature_mean_x = msg.data

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    lab3 = Lab3()
    lab3.spin()
    lab3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
