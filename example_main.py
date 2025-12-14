# example_main.py

from multi_coverage import MultiDronePlanner

def main():
    # Define your overall area
    # Format: (lat_min, lon_min, lat_max, lon_max)
    geofence = (12.8690, 80.2170, 12.8700, 80.2180)

    num_drones = 3
    altitude = 15
    camera_fov = 60
    overlap = 0.1  # 10% overlap

    planner = MultiDronePlanner(
        geofence=geofence,
        num_drones=num_drones,
        altitude=altitude,
        fov_deg=camera_fov,
        overlap=overlap
    )

    plans = planner.allocate()

    # Display and (in practice) send plans to drones
    for drone_id, plan in plans.items():
        region = plan["subregion"]
        wp_list = plan["waypoints"]

        print(f"Drone {drone_id} covers: {region}")
        print("Waypoints:")
        for idx, (lat, lon) in enumerate(wp_list):
            print(f"  {idx:3d}: {lat:.7f}, {lon:.7f}")
        print("-----\n")

    # TODO: send subregion and waypoints to each drone for execution
    # e.g. via MAVLink messages or a topic bus

if __name__ == "__main__":
    main()
