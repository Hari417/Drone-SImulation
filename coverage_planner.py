from math import ceil, tan, radians
from pyproj import Transformer

# Create transformers for converting between lat/lon and local planar (meters) coordinates
# We convert lat/lon => UTM automatically by using a dynamic projection
# Remember transformer expects (lon, lat) inputs and outputs (x, y)
transformer_to_xy = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
transformer_to_latlon = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

def latlon_to_xy(lat, lon):
    """
    Convert latitude/longitude to planar X, Y coordinates in meters.
    Uses a projection (Web Mercator EPSG:3857).
    """
    x, y = transformer_to_xy.transform(lon, lat)
    return x, y

def xy_to_latlon(x, y):
    """
    Convert planar X, Y meters back to latitude/longitude.
    """
    lon, lat = transformer_to_latlon.transform(x, y)
    return lat, lon

def compute_camera_swath(altitude_m, fov_deg):
    """
    Compute ground swath width (in meters) given altitude and field of view (degrees).
    """
    return 2 * altitude_m * tan(radians(fov_deg / 2))

def plan_lawnmower(geofence, altitude_m, camera_fov_deg, overlap=0):
    """
    Plan a lawnmower coverage path for a single drone.
    
    Args:
        geofence: tuple of (lat_min, lon_min, lat_max, lon_max)
        altitude_m: flight altitude (meters)
        camera_fov_deg: camera field of view (degrees)
        overlap: optional percent overlap between passes (0-1)
    
    Returns:
        List of (lat, lon) waypoints in sequence.
    """
    lat_min, lon_min, lat_max, lon_max = geofence

    # Convert corners to planar (meters)
    x_min, y_min = latlon_to_xy(lat_min, lon_min)
    x_max, y_max = latlon_to_xy(lat_max, lon_max)

    # Compute swath width reduced by overlap factor
    swath_width = compute_camera_swath(altitude_m, camera_fov_deg)
    effective_swath = swath_width * (1 - overlap)

    # Determine number of lanes needed
    area_width = abs(x_max - x_min)
    num_lanes = ceil(area_width / effective_swath)

    waypoints = []
    for i in range(num_lanes + 1):
        # Lane coordinate in planar X
        lane_x = x_min + i * effective_swath

        # Alternate direction for subsequent lanes
        if i % 2 == 0:
            # Bottom to top
            start_y = y_min
            end_y = y_max
        else:
            # Top to bottom
            start_y = y_max
            end_y = y_min

        # Convert each segment back to lat/lon
        lat1, lon1 = xy_to_latlon(lane_x, start_y)
        lat2, lon2 = xy_to_latlon(lane_x, end_y)
        waypoints.append((lat1, lon1))
        waypoints.append((lat2, lon2))

    return waypoints
