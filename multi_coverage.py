# multi_coverage.py

from partition import partition_rectangle_strip
from coverage_planner import plan_lawnmower

class MultiDronePlanner:
    def __init__(self, geofence, num_drones, altitude, fov_deg, overlap=0):
        """
        Args:
            geofence: (lat_min, lon_min, lat_max, lon_max)
            num_drones: how many drones
            altitude: coverage altitude
            fov_deg: camera field of view
            overlap: swath overlap [0..1)
        """
        self.geofence = geofence
        self.num_drones = num_drones
        self.altitude = altitude
        self.fov_deg = fov_deg
        self.overlap = overlap

    def allocate(self):
        """
        Partition the area and generate coverage plans for each drone.
        Returns:
            A dict of index -> coverage waypoints
            Example: {0: [(lat, lon), ...], 1: [...], ...}
        """
        subregions = partition_rectangle_strip(self.geofence, self.num_drones)
        plans = {}

        for idx, subregion in enumerate(subregions):
            # Plan lawnmower coverage for that subregion
            waypoints = plan_lawnmower(
                subregion,
                altitude_m=self.altitude,
                camera_fov_deg=self.fov_deg,
                overlap=self.overlap
            )
            plans[idx] = {
                "subregion": subregion,
                "waypoints": waypoints
            }

        return plans
