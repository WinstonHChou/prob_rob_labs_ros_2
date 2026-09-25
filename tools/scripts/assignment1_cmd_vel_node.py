import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


TARGET_TANGENTIAL_VELOCITY = 1.0
TARGET_RADIUS = 2.0
CMD_VEL_TOPIC = "cmd_vel"

class Assignment1CmdVelNode(Node):
    def __init__(self):
        super().__init__('assignment1_cmd_vel_node')
        self.publisher_ = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = TARGET_TANGENTIAL_VELOCITY
        msg.twist.angular.z = TARGET_TANGENTIAL_VELOCITY / TARGET_RADIUS / 2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    node = Assignment1CmdVelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()