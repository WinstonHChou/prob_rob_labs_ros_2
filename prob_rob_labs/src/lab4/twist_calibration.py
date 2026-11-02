import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt


PREFIX = "CALIBRATION"
HEARTBEAT_PERIOD = 0.1  # seconds
WAIT_FOR_MEAS    = 30.0 # seconds

STATE_COLLECT = 0
STATE_DONE = 1

class TwistCalibration(Node):
    def __init__(self):
        super().__init__('twist_calibration')

        self.gt_twist_sub = self.create_subscription(TwistStamped, 'gt_twist',
                                                     self.on_gt_twist, 10)
        self.gt_twist = None

        self.timer = self.create_timer(HEARTBEAT_PERIOD, self.heartbeat)
        self.first_heartbead_stamp = None
        self.state = None

        self.twist_lin_x_buffer = []
        self.twist_ang_z_buffer = []

        self.get_logger().info('Twist Calibration initialized.')

    def heartbeat(self):
        now = self.get_clock().now()

        # Initialize the state only if we have received a gt_twist value
        if self.first_heartbead_stamp is None:
            if self.gt_twist is None:
                return
            self.first_heartbead_stamp = now
            self.state = STATE_COLLECT
            self.get_logger().info(f'[{PREFIX}] state initialized to {self.state}.')
            return

        elapsed = (now - self.first_heartbead_stamp).nanoseconds / 1e9
        self.get_logger().info(f'[{PREFIX}] Elapsed time since last state change: {elapsed:.2f} seconds', throttle_duration_sec=5.0)

        if self.state == STATE_COLLECT:
            if elapsed >= WAIT_FOR_MEAS:
                self.state = STATE_DONE
                self.get_logger().info(f'[{PREFIX}] state changed to {self.state}.')
                self.first_heartbead_stamp = now
                return
        elif self.state == STATE_DONE:
            vel_x = np.array(self.twist_lin_x_buffer)
            ome_z = np.array(self.twist_ang_z_buffer)
            mean_vel_x = np.mean(vel_x)
            mean_ome_z = np.mean(ome_z)
            std_vel_x = np.std(vel_x, ddof=1)
            std_ome_z = np.std(ome_z, ddof=1)
        
            self.get_logger().info(
                f"[{PREFIX}] Calibration results:\n"
                f"Mean LINEAR X: {mean_vel_x:.3f} m/s, "
                f"Mean ANGULAR Z: {mean_ome_z:.3f} rad/s\n"
                f"Std LINEAR X: {std_vel_x:.3f} m/s, "
                f"Std ANGULAR Z: {std_ome_z:.3f} rad/s\n"
            )
            np.savez('twist_calibration.npz',
                     vel_x=vel_x,
                     ome_z=ome_z,
                     mean_vel_x=mean_vel_x,
                     mean_ome_z=mean_ome_z,
                     std_vel_x=std_vel_x,
                     std_ome_z=std_ome_z)
            self.twist_lin_x_buffer.clear()
            self.twist_ang_z_buffer.clear()
            raise KeyboardInterrupt

    def on_gt_twist(self, msg: TwistStamped):
        self.gt_twist = msg
        self.get_logger().info(f'[{PREFIX}] Received gt_twist: linear.x={msg.twist.linear.x:.3f}, angular.z={msg.twist.angular.z:.3f}', throttle_duration_sec=5.0)

        if not np.isfinite(msg.twist.linear.x) or not np.isfinite(msg.twist.angular.z):
            return

        # Since differential drive only have linear.x and angular.z in body twist
        if self.state == STATE_COLLECT:
            self.twist_lin_x_buffer.append(msg.twist.linear.x)
            self.twist_ang_z_buffer.append(msg.twist.angular.z)

def plot_calibration_results():
    NUM_BINS = 30

    try:
        data = np.load('twist_calibration.npz')
        vel_x = data['vel_x']
        ome_z = data['ome_z']
        mean_vel_x = float(data['mean_vel_x'])
        mean_ome_z = float(data['mean_ome_z'])
        std_vel_x = float(data['std_vel_x'])
        std_ome_z = float(data['std_ome_z'])
    except FileNotFoundError:
        print(f"[{PREFIX}] No calibration file found to plot.")
        return

    x_ax_vel_x = np.linspace(np.min(vel_x), np.max(vel_x), 200)
    x_ax_ome_z = np.linspace(np.min(ome_z), np.max(ome_z), 200)
    gaussian_vel_x = stats.norm.pdf(x_ax_vel_x, mean_vel_x, std_vel_x)
    gaussian_ome_z = stats.norm.pdf(x_ax_ome_z, mean_ome_z, std_ome_z)

    plt.figure(figsize=(10, 6))
    plt.subplot(1, 2, 1)

    plt.hist(
        vel_x, bins=np.histogram_bin_edges(vel_x, bins=NUM_BINS), density=True,
        alpha=0.4, label='Linear X Data', color='blue'
    )
    plt.axvline(mean_vel_x, color='green', linestyle='dashed', linewidth=1.5, label=f'Mean Linear X ({mean_vel_x:.1e}, {std_vel_x:.1e})')
    plt.plot(x_ax_vel_x, gaussian_vel_x, color='darkgreen', linewidth=2, label='Linear X Normal Curve')
    plt.xlabel('Linear Vel X (m/s)')
    plt.ylabel('Probability Density')
    plt.legend()

    plt.subplot(1, 2, 2)

    plt.hist(
        ome_z, bins=np.histogram_bin_edges(ome_z, bins=NUM_BINS), density=True,
        alpha=0.4, label='Angular Z Data', color='purple'
    )
    plt.axvline(mean_ome_z, color='red', linestyle='dashed', linewidth=1.5, label=f'Mean Angular Z ({mean_ome_z:.1e}, {std_ome_z:.1e})')
    plt.plot(x_ax_ome_z, gaussian_ome_z, color='darkred', linewidth=2, label='Angular Z Normal Curve')
    plt.xlabel('Angular rate Z (rad/s)')
    plt.ylabel('Probability Density')
    plt.legend()

    plt.title('Twist Calibration Distribution Results')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    plt.savefig('twist_calibration_results.png')
    plt.show()


def main():
    rclpy.init()
    node = TwistCalibration()
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
