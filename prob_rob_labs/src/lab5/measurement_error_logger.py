
import math
import csv

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from prob_rob_msgs.msg import LandmarkMeasurement, LandmarkMeasurementError


LANDMARK_X = 0.0 # For multiple bearings we have to adjust this in next lab to a lookup table
LANDMARK_Y = 0.0
CAMERA_OFFSET = 0.076  # meters, forward from robot origin
CSV_PATH = 'landmark_errors.csv'

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2)
    )

class MeasurementErrorLogger(Node):
    def __init__(self):
        super().__init__('measurement_error_logger')

        self.declare_parameter('landmark_x', LANDMARK_X)
        self.declare_parameter('landmark_y', LANDMARK_Y)
        self.declare_parameter('camera_offset', CAMERA_OFFSET)
        self.declare_parameter('csv_path', CSV_PATH)

        self.landmark_x = self.get_parameter('landmark_x').value
        self.landmark_y = self.get_parameter('landmark_y').value
        self.camera_offset = self.get_parameter('camera_offset').value
        csv_path = self.get_parameter('csv_path').value

        self._error_pub = self.create_publisher(
            LandmarkMeasurementError,
            '/landmark/measurement_error',
            10
        )

        self._measurement_sub = Subscriber(
            self,
            LandmarkMeasurement,
            '/landmark/measurement'
        )

        self._gt_sub = Subscriber(
            self,
            PoseStamped,
            '/tb3/ground_truth/pose'
        )

        # Match msgs from subscribers for timing
        self.sync = ApproximateTimeSynchronizer(
            [self._measurement_sub, self._gt_sub],
            queue_size=30,
            slop=0.08
        )
        self.sync.registerCallback(self._on_sync)

        # Saving outputs to a csv
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'measured_distance',
            'measured_bearing',
            'distance_error',
            'bearing_error'
        ])
        self.csv_file.flush()

        self.get_logger().info('Measurement error node started')

    def true_measurement(self, pose, landmark_x, landmark_y, camera_offset):
        # Robot ground truth pose in world frame
        xr = pose.position.x
        yr = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)

        # Camera position in world frame
        xc = xr + camera_offset * math.cos(yaw)
        yc = yr + camera_offset * math.sin(yaw)

        # delta camera to landmark in world frame
        dx = landmark_x - xc
        dy = landmark_y - yc

        # Transform to camera-aligned planar coordinates
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy # R(-yaw) * dp
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy

        distance = math.hypot(forward, left)
        bearing = math.atan2(left, forward)

        return distance, bearing

    def _on_sync(self, measurement, ground_truth):
        true_distance, true_bearing = self.true_measurement(
            ground_truth.pose,
            self.landmark_x,
            self.landmark_y,
            self.camera_offset
        )

        distance_error = measurement.distance - true_distance
        bearing_error = wrap_angle(
            measurement.bearing - true_bearing
        )

        error_msg = LandmarkMeasurementError()
        error_msg.measurement = measurement
        error_msg.distance_error = distance_error
        error_msg.bearing_error = bearing_error

        self._error_pub.publish(error_msg)

        timestamp = (
            measurement.header.stamp.sec
            + measurement.header.stamp.nanosec * 1e-9
        )

        self.csv_writer.writerow([
            timestamp,
            measurement.distance,
            measurement.bearing,
            distance_error,
            bearing_error
        ])
        self.csv_file.flush()

        self.get_logger().info(
            f'Distance: {measurement.distance:.3f} m, '
            f'Bearing: {measurement.bearing:.3f} rad, '
            f'Distance error: {distance_error:.3f} m, '
            f'Bearing error: {bearing_error:.4f} rad'
        )

    def destroy_node(self):
        self.csv_file.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = MeasurementErrorLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
