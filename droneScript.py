#!/usr/bin/env python3
"""
sitl_mission.py

Connects to an ArduPilot vehicle in Mission Planner SITL and:
 - Arms vehicle
 - Takes off to TARGET_ALT
 - Flies to two GPS waypoints
 - Yaws/loiters
 - Returns to launch and lands

Edit CONNECTION_STRING and waypoints as needed.
"""
import collections.abc
import collections
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command

import time
from math import radians, cos, sin, sqrt
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

# ======= CONFIG =======
# Example connection strings:
#  - 'udp:127.0.0.1:14550'
#  - 'tcp:127.0.0.1:5760'
#  - '127.0.0.1:14550' works for some setups (interpreted as udp)
CONNECTION_STRING = "tcp:127.0.0.1:5762"

TARGET_ALT = 10  # meters
GROUND_SPEED = 5  # m/s
HOME_WAIT_TIMEOUT = 30  # seconds
# Two sample waypoints (lat, lon). Replace with appropriate coords for your sim.
WAYPOINTS = [
    ( 12.869811, 80.217273 ),  # WP1
    ( 12.869616, 80.217289 ),  # WP2
]
# ======================


def connect_vehicle(conn_str, wait_ready=True, timeout=60):
    print(f"Connecting to vehicle on: {conn_str}")
    vehicle = connect(conn_str, wait_ready=wait_ready, timeout=timeout)
    print("Connected.")
    return vehicle


def arm_and_takeoff(vehicle, target_alt):
    """
    Arms vehicle and takes off to target_alt (meters).
    """
    print("Basic pre-arm checks")
    # Wait until vehicle is armable
    while not vehicle.is_armable:
        print(" Waiting for vehicle to be armable...")
        time.sleep(1)

    print("Changing to GUIDED mode")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(" Waiting for GUIDED mode...")
        time.sleep(0.5)

    print("Arming motors")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(0.5)

    print(f"Taking off to {target_alt} m")
    vehicle.simple_takeoff(target_alt)

    # Wait until the vehicle reaches a safe height before processing the next step.
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f} m")
        if alt >= target_alt * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def set_groundspeed(vehicle, speed_m_s):
    vehicle.groundspeed = speed_m_s
    # some vehicles respect param, but set for guidance
    print(f"Set ground speed to {speed_m_s} m/s")


def goto_location(vehicle, lat, lon, alt):
    """
    Fly to a global-relative coordinate.
    """
    target = LocationGlobalRelative(lat, lon, alt)
    print(f"Going to {lat}, {lon} @ {alt} m")
    vehicle.simple_goto(target)
    # Wait until we get near the target
    while True:
        loc = vehicle.location.global_relative_frame
        d = distance_meters(loc.lat, loc.lon, lat, lon)
        print(f" Distance to target: {d:.1f} m | Alt: {loc.alt:.1f} m")
        if d <= 2.0:
            print("Reached waypoint")
            break
        time.sleep(1)


def distance_meters(lat1, lon1, lat2, lon2):
    # Approx haversine for small distances — adequate for SITL
    R = 6371000.0
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2_safe(sqrt(a), sqrt(max(0.0, 1-a)))
    return R * c

def atan2_safe(y, x):
    # use math.atan2 but avoid any domain issues
    import math
    return math.atan2(y, x)

def condition_yaw(vehicle, heading, relative=False):
    """
    Set the yaw (heading) of the vehicle.
    heading: 0..360 degrees
    relative: True => yaw relative by heading degrees
    """
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,       # confirmation
        heading, # param 1: target angle
        0,       # param 2: speed deg/s
        1,       # param 3: direction -1 ccw, 1 cw
        is_relative, # param 4: relative offset 1, absolute 0
        0, 0, 0  # param 5-7 not used
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_ned_velocity(vehicle, vx, vy, vz, duration):
    """
    Move vehicle in local NED frame. vx, vy, vz in m/s. Duration in seconds.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # type mask: only velocities enabled
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # velocities
        0, 0, 0,  # acceleration
        0, 0)
    for _ in range(int(duration)):
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(1)


def main():
    vehicle = connect_vehicle(CONNECTION_STRING)

    try:
        # Ensure vehicle has a home location (SITL usually sets it quickly).
        if vehicle.location.global_frame.lat is None:
            print("Waiting for home location...")
            waited = 0
            while vehicle.location.global_frame.lat is None and waited < HOME_WAIT_TIMEOUT:
                time.sleep(1)
                waited += 1
            if vehicle.location.global_frame.lat is None:
                print("Warning: home location not received — continuing anyway.")

        set_groundspeed(vehicle, GROUND_SPEED)
        arm_and_takeoff(vehicle, TARGET_ALT)

        # Fly to waypoints
        for idx, (lat, lon) in enumerate(WAYPOINTS, start=1):
            print(f"WP {idx}")
            goto_location(vehicle, lat, lon, TARGET_ALT)
            print("Loitering for 5 seconds")
            time.sleep(5)

        # Yaw 360 degrees to face multiple directions (visual check in SITL)
        print("Rotating in place (yaw)")
        condition_yaw(vehicle, 360, relative=True)
        time.sleep(5)

        # Optional: forward velocity for a few seconds
        print("Moving forward for 5 seconds")
        send_ned_velocity(vehicle, 5, 0, 0, 5)

        # Return to launch
        print("Returning to Launch (RTL)")
        vehicle.mode = VehicleMode("RTL")
        while vehicle.mode.name != "RTL":
            print(" Waiting for RTL mode...")
            time.sleep(0.5)

        # Wait until disarmed (landed)
        print("Waiting for vehicle to land and disarm...")
        while vehicle.armed:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt:.1f} m | Armed: {vehicle.armed}")
            time.sleep(2)

        print("Mission complete. Vehicle disarmed.")

    except KeyboardInterrupt:
        print("User interrupted — switching to RTL")
        vehicle.mode = VehicleMode("RTL")
    finally:
        print("Closing vehicle connection")
        vehicle.close()


if __name__ == "__main__":
    main()
