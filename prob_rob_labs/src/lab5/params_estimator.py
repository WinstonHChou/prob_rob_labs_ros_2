from landmark_estimator import estimate_landmark


import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped, LandmarkMeasurement

from landmark_estimator import estimate_landmark


HEIGHT = 0.5 #meters
COLOR = 'cyan'
EPS = 1e-6
EDGE_MARGIN = 15  # pixels


class LandmarkMeasurementNode(Node):
    def __init__(self):
        super().__init__('landmark_measurement')

        self.declare_parameter('landmark_height', HEIGHT)
        self.declare_parameter('landmark_color', COLOR)

        self.height = self.get_parameter('landmark_height').value
        self.color = self.get_parameter('landmark_color').value

        self.fx = None
        self.fy = None
        self.cx = None

        self.image_width = None

        self.camera_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_callback,
            qos_profile_sensor_data
        )

        self.corners_sub = self.create_subscription(
            Point2DArrayStamped,
            f'/vision_{self.color}/corners',
            self.corners_callback,
            qos_profile_sensor_data
        )

        self.measurement_pub = self.create_publisher(
            LandmarkMeasurement,
            '/landmark/measurement',
            10
        )

    def camera_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]

        self.image_width = msg.width

    def corners_callback(self, msg):
        if self.fx is None or self.fy is None:
            return

        if not msg.points:
            return

        if self.image_width is None:
            return None

        x_min = min(p.x for p in msg.points)
        x_max = max(p.x for p in msg.points)

        if x_min <= EDGE_MARGIN or x_max >= self.image_width - EDGE_MARGIN:
            return

        result = estimate_landmark(msg.points)

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

        self.measurement_pub.publish(measurement)

        self.get_logger().info(
            f'Distance: {distance:.3f} m, '
            f'Bearing: {theta:.3f} rad'
        )


def main():
    rclpy.init()
    node = LandmarkMeasurementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
