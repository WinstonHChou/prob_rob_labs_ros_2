import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped

def estimate_landmark(corners):
    if not corners:
        return None

    x_min = x_max = corners[0].x
    y_min = y_max = corners[0].y

    for point in corners[1:]:
        x_min = min(x_min, point.x)
        x_max = max(x_max, point.x)
        y_min = min(y_min, point.y)
        y_max = max(y_max, point.y)

    height = y_max - y_min
    center_x = (x_min + x_max) / 2

    return height, center_x

class LandmarkEstimator(Node):
    def __init__(self):
        super().__init__('landmark_estimator')
        self.subscription = self.create_subscription(
            Point2DArrayStamped,
            '/vision_cyan/corners',
            self.callback,
            10
        )

    def callback(self, msg):
        result = estimate_landmark(msg.points)

        if result is None: # Robot is not facing cyan landmark
            return

        height, center_x = result

        self.get_logger().info(
            f'Height: {height:.1f} px, Axis: {center_x:.1f} px'
        )


def main():
    rclpy.init()
    node = LandmarkEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()