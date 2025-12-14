# partition.py

from math import ceil
from pyproj import Transformer

# Setup a projection for meter-based calculations
transformer_to_xy = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
transformer_to_latlon = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

def latlon_to_xy(lat, lon):
    x, y = transformer_to_xy.transform(lon, lat)
    return x, y

def xy_to_latlon(x, y):
    lon, lat = transformer_to_latlon.transform(x, y)
    return lat, lon

def partition_rectangle_strip(geofence, num_drones):
    """
    Partitions a rectangular geofence into vertical strips (equal width).
    Args:
        geofence: (lat_min, lon_min, lat_max, lon_max)
        num_drones: number of drones
    Returns:
        List of geofence subregions
        Each subregion is (lat_min, lon_min, lat_max, lon_max)
    """
    lat_min, lon_min, lat_max, lon_max = geofence

    # Convert corners to planar (meters)
    x_min, y_min = latlon_to_xy(lat_min, lon_min)
    x_max, y_max = latlon_to_xy(lat_max, lon_max)

    total_width = abs(x_max - x_min)
    strip_width = total_width / num_drones
    subregions = []

    for i in range(num_drones):
        left = x_min + i * strip_width
        right = left + strip_width

        # Convert back to lat/lon
        lat1, lon1 = xy_to_latlon(left, y_min)
        lat2, lon2 = xy_to_latlon(right, y_max)

        subregions.append((lat1, lon1, lat2, lon2))

    return subregions
