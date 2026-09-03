# pyrefly: ignore [missing-import]
import numpy as np

def laser_scan_to_xy(scan):
    # Converts a LaserScan message into a list of valid (x, y) coordinates in the sensor frame.
    ranges = np.asarray(scan.ranges, dtype=np.float32)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    ranges = ranges[valid]
    angles = angles[valid]

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    points = np.column_stack((x, y))

    return points, angles


def compute_rmse(distances):
    # Calculates Root Mean Square Error (RMSE) to quantify the overall scan-to-map mismatch.
    if len(distances) == 0:
        return np.nan
    return float(np.sqrt(np.mean(distances ** 2)))


def compute_sector_rmse(distances, angles, num_sectors=8):
    # Divides the scan into angular sectors and calculates RMSE for each independently.
    result = np.full(num_sectors, np.nan, dtype=np.float32)
    sector_width = 2.0 * np.pi / num_sectors

    normalized_angles = (angles + np.pi) % (2.0 * np.pi)
    sector_ids = (normalized_angles / sector_width).astype(np.int32)

    for sector in range(num_sectors):
        mask = sector_ids == sector
        if np.sum(mask) >= 10:  # Require at least 10 points to consider a sector valid
            result[sector] = np.sqrt(np.mean(distances[mask] ** 2))

    return result


def extract_amcl_covariance(msg):
    # Returns combined standard deviation in XY
    cov = msg.pose.covariance
    return np.sqrt(cov[0] + cov[7])


def classify_state(sector_rmses, std_xy, rmse_localized, rmse_degraded, std_xy_limit):
    valid_rmses = sector_rmses[~np.isnan(sector_rmses)]
    
    if len(valid_rmses) < 4:
        return "DEGRADED"  # Too many blind sectors
        
    if std_xy >= std_xy_limit:
        return "DEGRADED"

    degraded_sectors = np.sum(valid_rmses >= rmse_localized)
    lost_sectors = np.sum(valid_rmses >= rmse_degraded)
    
    if lost_sectors >= 2 or degraded_sectors >= 3:
        return "LOST"
        
    if degraded_sectors >= 1 or lost_sectors >= 1:
        return "DEGRADED"
        
    return "LOCALIZED"


def transform_points(points, transform):
    # Transforms 2D points using a ROS geometry_msgs/Transform
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y

    q = transform.transform.rotation
    yaw = np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2)
    )

    c = np.cos(yaw)
    s = np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array([tx, ty])

    return points @ rotation.T + translation
