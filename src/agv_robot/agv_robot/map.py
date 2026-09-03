# pyrefly: ignore [missing-import]
import numpy as np
# pyrefly: ignore [missing-import]
from scipy.ndimage import distance_transform_edt


class MapModel:
    # Manages the static 2D occupancy grid and converts coordinates.

    def __init__(self, occupancy, resolution, origin_x, origin_y):
        self.occupancy = occupancy
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    def world_to_map(self, points):
        # Converts real-world (x, y) coordinates to map cell indices (col, row).
        points = np.asarray(points)
        map_x = ((points[:, 0] - self.origin_x) / self.resolution).astype(np.int32)
        map_y = ((points[:, 1] - self.origin_y) / self.resolution).astype(np.int32)
        return map_x, map_y

    def valid_indices(self, map_x, map_y):
        # Filters out points that fall outside the map boundaries.
        return (
            (map_x >= 0) &
            (map_x < self.occupancy.shape[1]) &
            (map_y >= 0) &
            (map_y < self.occupancy.shape[0])
        )


def build_distance_field(occupancy, resolution):
    # Computes the Euclidean distance from every free cell to the nearest obstacle.
    occupied = occupancy >= 50
    free = ~occupied
    distance_cells = distance_transform_edt(free)
    distance_meters = distance_cells * resolution
    return distance_meters
