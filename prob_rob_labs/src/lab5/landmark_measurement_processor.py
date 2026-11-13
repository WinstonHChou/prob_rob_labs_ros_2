import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped, LandmarkMeasurement


HEIGHT = 0.5 # meters
COLOR = 'cyan'
EPS = 1e-6
EDGE_MARGIN = 15  # pixels


class LandmarkMeasurementProcessor(Node):
    def __init__(self):
        super().__init__('landmark_measurement_processor')

        self.declare_parameter('landmark_height', HEIGHT)
        self.declare_parameter('landmark_color', COLOR)

        self.height = self.get_parameter('landmark_height').value
        self.color = self.get_parameter('landmark_color').value

        self.fx = None
        self.fy = None
        self.cx = None

        self.image_width = None
        self.image_height = None

        self._camera_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self._on_camera,
            qos_profile_sensor_data
        )

        self._corners_sub = self.create_subscription(
            Point2DArrayStamped,
            f'/vision_{self.color}/corners',
            self._on_corners,
            qos_profile_sensor_data
        )

        self._measurement_pub = self.create_publisher(
            LandmarkMeasurement,
            '/landmark/measurement',
            10
        )

    def estimate_center(self, corners):
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

    def _on_camera(self, msg):
        self.fx = msg.p[0]
        self.fy = msg.p[5]
        self.cx = msg.p[2]
        self.image_width = msg.width
        self.image_height = msg.height

    def _on_corners(self, msg):
        if self.fx is None or self.fy is None or self.image_width is None:
            return

        if not msg.points:
            return

        x_min = min(p.x for p in msg.points)
        x_max = max(p.x for p in msg.points)

        if x_min <= EDGE_MARGIN or x_max >= self.image_width - EDGE_MARGIN:
            return

        result = self.estimate_center(msg.points)
        if result is None:
            return

        pixel_height, xp = result
        if pixel_height <= 0 or self.fx <= 0 or self.fy <= 0:
            return

        # Calculate parameters 
        theta = math.atan((self.cx - xp) / self.fx)

        denominator = pixel_height * math.cos(theta)
        if abs(denominator) < EPS:
            return

        distance = self.height * self.fy / denominator

        measurement = LandmarkMeasurement()
        measurement.header = msg.header
        measurement.distance = float(distance)
        measurement.bearing = float(theta)

        self._measurement_pub.publish(measurement)

        self.get_logger().info(
            f'Distance: {distance:.3f} m, '
            f'Bearing: {theta:.3f} rad'
        )


def main():
    rclpy.init()
    node = LandmarkMeasurementProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
