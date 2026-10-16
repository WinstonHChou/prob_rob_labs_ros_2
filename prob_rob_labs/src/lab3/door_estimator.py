import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Empty

import numpy as np

GOOD_BEL_THRESHOLD = 0.9999
FEATURE_MEAN_X_THRESHOLD = 321.878 # px (260 without firetruck, 321.878 with firetruck)

DOOR_OPEN_IDX   = 0
DOOR_CLOSED_IDX = 1

BEL_INIT = np.array([[0.5], [0.5]])  # Initial belief: 50% open, 50% closed

# Motion model from measured result
OBSERVED_PUSH_SUCCESS_RATE = 7/27
P_F_PUSH = np.array([
    [1.0, OBSERVED_PUSH_SUCCESS_RATE],
    [0.0, 1.0 - OBSERVED_PUSH_SUCCESS_RATE]
])

# Collected from calibration results
P_H = np.array([
    [0.9562, 0.1039],
    [0.0438, 0.8961]
])

class DoorEstimator(Node):
    def __init__(self):
        super().__init__('door_estimator')

        self.door_open_sub = self.create_subscription(Empty, 'door_open',
                                                      self.on_door_open_cmd, 10)
        self.feature_mean_x_sub = self.create_subscription(Float64, 'feature_mean_x',
                                                           self.on_feature_mean_x, 10)

        self.is_door_opened_pub = self.create_publisher(Bool, 'is_door_opened', 10)

        self.bel = BEL_INIT.copy()
        self.get_logger().info('Door Estimator initialized.')

    def on_door_open_cmd(self, msg):
        # Predict belief based on the motion model
        self.bel = self.predict(self.bel)

    def on_feature_mean_x(self, msg):
        feature_mean_x = msg.data
        if not np.isfinite(feature_mean_x):
            return

        self.bel = self.update(self.bel, feature_mean_x)
        p_open = float(self.bel[DOOR_OPEN_IDX, 0])
        p_closed = float(self.bel[DOOR_CLOSED_IDX, 0])
        ready = p_open >= GOOD_BEL_THRESHOLD

        self.is_door_opened_pub.publish(Bool(data=ready))

        self.get_logger().info(
            f'feature_mean={feature_mean_x:.3f}, '
            f'P(open)={p_open:.6f}, '
            f'P(closed)={p_closed:.6f}, '
            f'ready={ready}'
        )

    def predict(self, bel):
        # Predict belief based on the motion model
        bel_pred = P_F_PUSH @ bel
        return bel_pred

    def update(self, bel, feature_mean_x):
        # Compute the likelihood of the observation given the door state
        z_door_open = int(feature_mean_x >= FEATURE_MEAN_X_THRESHOLD)

        # Update the belief using Bayes' rule
        unnorm_bel_posterior = \
            (P_H * np.repeat(bel, 2, axis=1).transpose())[z_door_open]
        bel_posterior = np.array([unnorm_bel_posterior / sum(unnorm_bel_posterior)]).T

        return bel_posterior

def main():
    rclpy.init()
    node = DoorEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
