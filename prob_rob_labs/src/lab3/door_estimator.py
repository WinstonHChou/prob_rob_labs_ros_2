import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped


HEARTBEAT_PERIOD = 0.1  # seconds

FEATURE_MEAN_X_THRESHOLD = 260 # px

TORQUE_DOOR_OPEN = 5.0    # N*m

DOOR_OPEN_IDX   = 0
DOOR_CLOSED_IDX = 1

class DoorEstimator(Node):
    def __init__(self):
        super().__init__('door_estimator')

        self.feature_mean_x_sub = self.create_subscription(Float64, 'feature_mean_x',
                                                           self.on_feature_mean_x, 10)
        self.feature_mean_x = None

        self.door_torque_pub = self.create_publisher(Float64, 'door_torque', 10)
        # self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.timer = self.create_timer(HEARTBEAT_PERIOD, self.heartbeat)
        self.last_state_stamp = None

        self.get_logger().info('Door Estimator initialized.')

    def heartbeat(self):
        now = self.get_clock().now()

        # Initialize the state if it hasn't been set yet
        if self.last_state_stamp is None:
            self.last_state_stamp = now
            self.get_logger().info(f'state initialized to {self.state}.')
            return

        elapsed = (now - self.last_state_stamp).nanoseconds / 1e9
        # self.get_logger().info(f'Elapsed time since last state change: {elapsed:.2f} seconds')


    def on_feature_mean_x(self, msg):
        self.feature_mean_x = msg.data

def main():
    rclpy.init()
    node = DoorEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
