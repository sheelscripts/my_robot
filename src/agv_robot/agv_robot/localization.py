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
from .metrics import laser_scan_to_xy, compute_rmse, compute_sector_rmse, extract_amcl_covariance, classify_state, transform_points

class LocalizationHealthNode(Node):
    def __init__(self):
        super().__init__("localization_health")
        self.declare_parameters("", [
            ("rmse_localized_threshold", 0.50), ("rmse_degraded_threshold", 0.80),
            ("std_dev_xy_threshold", 1.0), ("lost_persistence", 5), ("recovery_persistence", 5)
        ])
        
        self.map_model, self.distance_field, self.latest_amcl = None, None, None
        self.current_state, self.lost_counter, self.recovery_counter = "UNKNOWN", 0, 0
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.state_pub = self.create_publisher(String, "/localization_health/state", 10)
        
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda msg: setattr(self, 'latest_amcl', msg), 10)

    def map_callback(self, msg):
        occupancy = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_model = MapModel(occupancy=occupancy, resolution=msg.info.resolution, origin_x=msg.info.origin.position.x, origin_y=msg.info.origin.position.y)
        self.distance_field = build_distance_field(occupancy, msg.info.resolution)

    def scan_callback(self, scan):
        if not self.map_model or not self.latest_amcl:
            return self.get_logger().info("Waiting for /map and /amcl_pose...", throttle_duration_sec=2.0)

        try:
            transform = self.tf_buffer.lookup_transform("map", scan.header.frame_id, scan.header.stamp, timeout=Duration(seconds=0.05))
        except Exception as e:
            return self.get_logger().warn(f"TF Error: {e}", throttle_duration_sec=2.0)

        points, angles = laser_scan_to_xy(scan)
        if len(points) < 50: return

        mx, my = self.map_model.world_to_map(transform_points(points, transform))
        valid_idx = self.map_model.valid_indices(mx, my)
        
        # Points that fall outside the map are given a large penalty distance (2.0m)
        distances = np.full(len(points), 2.0, dtype=np.float32)
        distances[valid_idx] = self.distance_field[my[valid_idx], mx[valid_idx]]
        
        global_rmse = compute_rmse(distances)
        
        # Get parameter 'num_sectors' safely or default to 8
        num_sectors = self.get_parameter_or("num_sectors", rclpy.parameter.Parameter("num_sectors", rclpy.Parameter.Type.INTEGER, 8)).value
        sector_rmses = compute_sector_rmse(distances, angles, num_sectors=num_sectors)
        
        std_xy = extract_amcl_covariance(self.latest_amcl)

        p = {k: self.get_parameter(k).value for k in ["rmse_localized_threshold", "rmse_degraded_threshold", "std_dev_xy_threshold", "lost_persistence", "recovery_persistence"]}
        raw_state = classify_state(sector_rmses, std_xy, p["rmse_localized_threshold"], p["rmse_degraded_threshold"], p["std_dev_xy_threshold"])

        self.lost_counter = self.lost_counter + 1 if raw_state == "LOST" else 0
        self.recovery_counter = self.recovery_counter + 1 if raw_state == "LOCALIZED" else 0

        if self.lost_counter >= p["lost_persistence"]: self.current_state = "LOST"
        elif self.recovery_counter >= p["recovery_persistence"]: self.current_state = "LOCALIZED"
        elif raw_state == "DEGRADED": self.current_state = "DEGRADED"

        self.state_pub.publish(String(data=self.current_state))

        self.get_logger().info(f"State: {self.current_state} (Raw: {raw_state}) | Global RMSE: {global_rmse:.3f} | Bad Sectors: {np.sum(sector_rmses >= p['rmse_localized_threshold'])} | StdXY: {std_xy:.3f}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalizationHealthNode())
    rclpy.shutdown()

if __name__ == "__main__": main()
