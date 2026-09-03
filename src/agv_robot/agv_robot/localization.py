# pyrefly: ignore [missing-import]
import rclpy, numpy as np
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
# pyrefly: ignore [missing-import]
from rclpy.duration import Duration
# pyrefly: ignore [missing-import]
import tf2_ros
# pyrefly: ignore [missing-import]
from nav_msgs.msg import OccupancyGrid
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import LaserScan
# pyrefly: ignore [missing-import]
from geometry_msgs.msg import PoseWithCovarianceStamped
# pyrefly: ignore [missing-import]
from std_msgs.msg import String

from .map import MapModel, build_distance_field
from .metrics import (
    laser_scan_to_xy, compute_rmse, compute_sector_rmse,
    extract_amcl_covariance, classify_state, transform_points
)


class LocalizationHealthNode(Node):
    def __init__(self):
        super().__init__("localization_health")
        self.declare_parameters("", [
            ("num_sectors", 8),
            ("rmse_localized_threshold", 0.50),
            ("rmse_degraded_threshold", 0.80),
            ("std_dev_xy_threshold", 1.0),
            ("lost_persistence", 5),
            ("recovery_persistence", 5),
        ])

        self.map_model = None
        self.distance_field = None
        self.latest_amcl = None
        self.current_state = "UNKNOWN"
        self.lost_counter = 0
        self.recovery_counter = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(String, "/localization_health/state", 10)

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)

    def amcl_callback(self, msg):
        self.latest_amcl = msg

    def map_callback(self, msg):
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.map_model = MapModel(grid, info.resolution, info.origin.position.x, info.origin.position.y)
        self.distance_field = build_distance_field(grid, info.resolution)

    def scan_callback(self, scan):
        if not self.map_model or not self.latest_amcl:
            return self.get_logger().info("Waiting for /map and /amcl_pose...", throttle_duration_sec=2.0)

        try:
            tf = self.tf_buffer.lookup_transform("map", scan.header.frame_id, scan.header.stamp, timeout=Duration(seconds=0.05))
        except Exception as e:
            return self.get_logger().warn(f"TF Error: {e}", throttle_duration_sec=2.0)

        pts, angles = laser_scan_to_xy(scan)
        if len(pts) < 50:
            return

        mx, my = self.map_model.world_to_map(transform_points(pts, tf))
        valid = self.map_model.valid_indices(mx, my)

        distances = np.full(len(pts), 2.0, dtype=np.float32)
        distances[valid] = self.distance_field[my[valid], mx[valid]]

        num_sec = self.get_parameter("num_sectors").value
        th_loc = self.get_parameter("rmse_localized_threshold").value
        th_deg = self.get_parameter("rmse_degraded_threshold").value
        th_std = self.get_parameter("std_dev_xy_threshold").value
        n_lost = self.get_parameter("lost_persistence").value
        n_rec = self.get_parameter("recovery_persistence").value

        sector_rmses = compute_sector_rmse(distances, angles, num_sectors=num_sec)
        std_xy = extract_amcl_covariance(self.latest_amcl)
        raw_state = classify_state(sector_rmses, std_xy, th_loc, th_deg, th_std)

        self.lost_counter = self.lost_counter + 1 if raw_state == "LOST" else 0
        self.recovery_counter = self.recovery_counter + 1 if raw_state == "LOCALIZED" else 0

        if self.lost_counter >= n_lost:
            self.current_state = "LOST"
        elif self.recovery_counter >= n_rec:
            self.current_state = "LOCALIZED"
        elif raw_state == "DEGRADED":
            self.current_state = "DEGRADED"

        self.state_pub.publish(String(data=self.current_state))
        bad_sectors = int(np.sum(sector_rmses >= th_loc))
        self.get_logger().info(
            f"State: {self.current_state} (Raw: {raw_state}) | Global RMSE: {compute_rmse(distances):.3f} | Bad Sectors: {bad_sectors} | StdXY: {std_xy:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalizationHealthNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
