from Controls import DroneController
from coverage_planner import plan_lawnmower

# Define geofence
geo = (12.8690, 80.2170, 12.8700, 80.2180)

# Flight planning parameters
ALT = 15        # altitude (meters)
FOV = 60        # camera field of view (degrees)
OVERLAP = 0.2   # 20% overlap between swaths

# Create waypoints for coverage
waypoints = plan_lawnmower(geo, ALT, FOV, overlap=OVERLAP)

# Connect and fly
drone = DroneController("tcp:127.0.0.1:5762", default_alt=ALT)
drone.connect()

# Optional: takeoff before coverage
drone.takeoff(ALT)

# Fly coverage path
for lat, lon in waypoints:
    print(f"Going to {lat}, {lon}")
    drone.goto(lat, lon)

# Land at end of coverage
drone.land()
drone.disconnect()
