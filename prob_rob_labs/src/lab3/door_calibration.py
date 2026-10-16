import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt


PREFIX = "CALIBRATION"
HEARTBEAT_PERIOD = 0.1  # seconds
WAIT_FOR_MEAS    = 30.0 # seconds
WAIT_FOR_TRANSITION = 5.0 # seconds

TORQUE_DOOR_OPEN = 5.0    # N*m

STATE_DOOR_OPEN   = "OPEN"
STATE_DOOR_TRANSITION = "TRANSITION"
STATE_DOOR_CLOSED = "CLOSED"
STATE_CALIBRATING   = "CALIBRATING"

def choose_threshold(open_samples, closed_samples):
    o = np.asarray(open_samples, dtype=float)
    c = np.asarray(closed_samples, dtype=float)

    values = np.unique(np.concatenate([o, c]))
    thresholds = np.r_[
        -np.inf,
        values[:-1] + np.diff(values) / 2,
        np.inf,
    ]

    errors = np.array([
        0.5 * (np.mean(o >= t) + np.mean(c < t))
        for t in thresholds
    ])

    i = np.argmin(errors)
    return thresholds[i], errors[i]

class DoorCalibration(Node):
    def __init__(self):
        super().__init__('door_calibration')

        self.feature_mean_x_sub = self.create_subscription(Float64, 'feature_mean_x',
                                                           self.on_feature_mean_x, 10)
        self.feature_mean_x = None

        self.door_torque_pub = self.create_publisher(Float64, 'door_torque', 10)

        self.timer = self.create_timer(HEARTBEAT_PERIOD, self.heartbeat)
        self.first_heartbead_stamp = None
        self.state = None

        self.feature_mean_open_buffer = []
        self.feature_mean_closed_buffer = []

        self.get_logger().info('Door Calibration initialized.')

    def heartbeat(self):
        now = self.get_clock().now()

        # Initialize the state only if we have received a feature_mean_x value
        if self.first_heartbead_stamp is None:
            if self.feature_mean_x is None:
                return
            self.first_heartbead_stamp = now
            self.state = STATE_DOOR_CLOSED
            self.get_logger().info(f'[{PREFIX}] state initialized to {self.state}.')
            return

        elapsed = (now - self.first_heartbead_stamp).nanoseconds / 1e9
        self.get_logger().info(f'[{PREFIX}] Elapsed time since last state change: {elapsed:.2f} seconds', throttle_duration_sec=5.0)

        if self.state == STATE_DOOR_CLOSED:
            if elapsed >= WAIT_FOR_MEAS:
                self.state = STATE_DOOR_TRANSITION
                self.get_logger().info(f'[{PREFIX}] state changed to {self.state}.')
                self.first_heartbead_stamp = now
                return
        elif self.state == STATE_DOOR_TRANSITION:
            if elapsed >= WAIT_FOR_TRANSITION:
                self.state = STATE_DOOR_OPEN
                self.get_logger().info(f'[{PREFIX}] state changed to {self.state}.')
                self.first_heartbead_stamp = now
                return
            torque_msg = Float64()
            torque_msg.data = TORQUE_DOOR_OPEN
            self.door_torque_pub.publish(torque_msg)
        elif self.state == STATE_DOOR_OPEN:
            if elapsed >= WAIT_FOR_MEAS:
                self.state = STATE_CALIBRATING
                self.get_logger().info(f'[{PREFIX}] state changed to {self.state}.')
                self.first_heartbead_stamp = now
                return
        elif self.state == STATE_CALIBRATING:
            open_data = np.array(self.feature_mean_open_buffer)
            closed_data = np.array(self.feature_mean_closed_buffer)
            mean_open = np.mean(open_data)
            mean_closed = np.mean(closed_data)
            std_open = np.std(open_data, ddof=1)
            std_closed = np.std(closed_data, ddof=1)
            recommended_threshold, error_rate = choose_threshold(open_data, closed_data)

            n_open = open_data.size
            n_closed = closed_data.size
            n_open_open = np.count_nonzero(open_data < recommended_threshold)
            n_closed_open = n_open - n_open_open
            n_open_closed = np.count_nonzero(closed_data < recommended_threshold)
            n_closed_closed = n_closed - n_open_closed

            p_open_open = n_open_open / n_open
            p_closed_open = n_closed_open / n_open
            p_open_closed = n_open_closed / n_closed
            p_closed_closed = n_closed_closed / n_closed
        
            self.get_logger().info(
                f"[{PREFIX}] Calibration results:\n"
                f"Mean OPEN: {mean_open:.3f} px, "
                f"Mean CLOSED: {mean_closed:.3f} px\n"
                f"Std OPEN: {std_open:.3f} px, "
                f"Std CLOSED: {std_closed:.3f} px\n"
                f"Threshold: {recommended_threshold:.3f} px\n"
                f"Balanced calibration error: {error_rate:.2%}\n\n"
                f"Actual OPEN: N={n_open}, "
                f"detected open={n_open_open}, "
                f"detected closed={n_closed_open}\n"
                f"Actual CLOSED: N={n_closed}, "
                f"detected open={n_open_closed}, "
                f"detected closed={n_closed_closed}\n\n"
                f"P(z=open | x=open)     = {p_open_open:.4f}\n"
                f"P(z=closed | x=open)   = {p_closed_open:.4f}\n"
                f"P(z=open | x=closed)   = {p_open_closed:.4f}\n"
                f"P(z=closed | x=closed) = {p_closed_closed:.4f}"
            )
            np.savez('door_calibration.npz',
                     open_data=open_data,
                     closed_data=closed_data,
                     mean_open=mean_open, mean_closed=mean_closed,
                     std_open=std_open, std_closed=std_closed,
                     recommended_threshold=recommended_threshold,
                     error_rate=error_rate)
            self.feature_mean_open_buffer.clear()
            self.feature_mean_closed_buffer.clear()
            raise KeyboardInterrupt

    def on_feature_mean_x(self, msg):
        self.feature_mean_x = float(msg.data)

        if not np.isfinite(self.feature_mean_x):
            return

        if self.state == STATE_DOOR_CLOSED:
            self.feature_mean_closed_buffer.append(self.feature_mean_x)
        elif self.state == STATE_DOOR_OPEN:
            self.feature_mean_open_buffer.append(self.feature_mean_x)

def plot_calibration_results():
    try:
        data = np.load('door_calibration.npz')
        open_data = data['open_data']
        closed_data = data['closed_data']
        mean_open = float(data['mean_open'])
        mean_closed = float(data['mean_closed'])
        std_open = float(data['std_open'])
        std_closed = float(data['std_closed'])
        recommended_threshold = float(data['recommended_threshold'])
        error_rate = float(data['error_rate'])
    except FileNotFoundError:
        print(f"[{PREFIX}] No calibration file found to plot.")
        return

    plt.figure(figsize=(10, 6))
    all_data = np.concatenate([open_data, closed_data])
    bins = np.histogram_bin_edges(all_data, bins=30)

    hist_open, _, _ = plt.hist(
        open_data, bins=bins, density=True,
        alpha=0.4, label='Open Data', color='green'
    )

    hist_closed, _, _ = plt.hist(
        closed_data, bins=bins, density=True,
        alpha=0.4, label='Closed Data', color='red'
    )

    overlap = np.minimum(hist_open, hist_closed)
    plt.stairs(
        overlap,
        bins,
        baseline=0,
        fill=True,
        facecolor='purple',
        edgecolor='purple',
        alpha=0.5,
        hatch='///',
        linewidth=1,
        label='Histogram Overlap',
        zorder=3,
    )

    plt.axvline(mean_open, color='green', linestyle='dashed', linewidth=1.5, label=f'Mean Open ({mean_open:.1f})')
    plt.axvline(mean_closed, color='red', linestyle='dashed', linewidth=1.5, label=f'Mean Closed ({mean_closed:.1f})')
    plt.axvline(recommended_threshold, color='blue', linestyle='solid', linewidth=2, label=f'Recommended Threshold ({recommended_threshold:.1f}, err rate: {error_rate:.2%})')

    all_data = np.concatenate([open_data, closed_data])
    x_axis = np.linspace(np.min(all_data) - 5, np.max(all_data) + 5, 200)
    gaussian_open = stats.norm.pdf(x_axis, mean_open, std_open)
    gaussian_closed = stats.norm.pdf(x_axis, mean_closed, std_closed)
    
    plt.plot(x_axis, gaussian_open, color='darkgreen', linewidth=2, label='Open Normal Curve')
    plt.plot(x_axis, gaussian_closed, color='darkred', linewidth=2, label='Closed Normal Curve')

    plt.title('Door Calibration Distribution Results')
    plt.xlabel('Feature Mean X (px)')
    plt.ylabel('Probability Density')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig('door_calibration_results.png')
    plt.show()


def main():
    rclpy.init()
    node = DoorCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    plot_calibration_results()


if __name__ == '__main__':
    main()
