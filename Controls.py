import collections.abc
import collections
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from math import radians, cos, sin, sqrt, atan2

class DroneController:
    def __init__(self, connection_string, default_alt=10, default_speed=5):
        """
        Initialize controller with MAVLink connection string.
        Args:
            connection_string (str): e.g. "tcp:127.0.0.1:5762" or "udp:127.0.0.1:14550"
            default_alt (float): default takeoff altitude (meters)
            default_speed (float): default groundspeed (m/s)
        """
        self.conn_str = connection_string
        self.default_alt = default_alt
        self.default_speed = default_speed
        self.vehicle = None

    # -----------------------------
    # CONNECTION / DISCONNECTION
    # -----------------------------
    def connect(self, wait_ready=True, timeout=60):
        print(f"[Controls] Connecting to: {self.conn_str}")
        self.vehicle = connect(self.conn_str, wait_ready=wait_ready, timeout=timeout)
        print("[Controls] Vehicle connected.")
        return self.vehicle

    def disconnect(self):
        if self.vehicle:
            print("[Controls] Closing connection.")
            self.vehicle.close()
            self.vehicle = None

    # -----------------------------
    # STATE & MODE CONTROL
    # -----------------------------
    def set_mode(self, mode):
        """
        Change flight mode.
        Example: "GUIDED", "RTL", "LAND", "AUTO"
        """
        if not self.vehicle:
            raise RuntimeError("Vehicle not connected")
        print(f"[Controls] Setting mode: {mode}")
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            time.sleep(0.2)

    def arm(self):
        """Arms the vehicle."""
        print("[Controls] Arming motors")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(0.2)

    def disarm(self):
        """Disarms the vehicle."""
        print("[Controls] Disarming motors")
        self.vehicle.armed = False
        while self.vehicle.armed:
            time.sleep(0.2)

    # -----------------------------
    # BASIC FLIGHT
    # -----------------------------
    def takeoff(self, altitude=None):
        """
        Arms (if needed) and takes off to target altitude.
        """
        if altitude is None:
            altitude = self.default_alt

        print(f"[Controls] Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle armable...")
            time.sleep(0.5)

        self.set_mode("GUIDED")
        self.arm()

        print(f"[Controls] Taking off to {altitude} m")
        self.vehicle.simple_takeoff(altitude)

        # wait until reach altitude
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= altitude * 0.95:
                print("[Controls] Reached target altitude")
                break
            time.sleep(0.5)

    def land(self):
        """Lands the vehicle."""
        print("[Controls] Landing")
        self.set_mode("LAND")

    def return_to_launch(self):
        """Return to launch (RTL)."""
        print("[Controls] RTL")
        self.set_mode("RTL")

    # -----------------------------
    # POSITION CONTROL
    # -----------------------------
    def set_speed(self, speed):
        """
        Set groundspeed.
        """
        print(f"[Controls] Setting groundspeed: {speed} m/s")
        self.vehicle.groundspeed = speed

    def goto(self, lat, lon, alt=None):
        """
        Goto by GPS coordinates.
        """
        if alt is None:
            alt = self.default_alt
        print(f"[Controls] Goto: {lat}, {lon}, {alt}")
        pt = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(pt)

        # wait until reached (approx)
        while True:
            loc = self.vehicle.location.global_relative_frame
            d = self._distance_meters(loc.lat, loc.lon, lat, lon)
            if d < 2.0:
                print("[Controls] Waypoint reached")
                break
            time.sleep(0.5)

    # -----------------------------
    # VELOCITY CONTROL
    # -----------------------------
    def set_velocity(self, vx, vy, vz, duration=1):
        """
        Move in body-frame or NED frame by velocity.
        vx, vy, vz in m/s
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        for _ in range(int(duration)):
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            time.sleep(1)

    def move_forward(self, speed=1, duration=1):
        print("[Controls] Move forward")
        self.set_velocity(speed, 0, 0, duration)

    def move_backward(self, speed=1, duration=1):
        print("[Controls] Move backward")
        self.set_velocity(-speed, 0, 0, duration)

    def move_right(self, speed=1, duration=1):
        print("[Controls] Move right")
        self.set_velocity(0, speed, 0, duration)

    def move_left(self, speed=1, duration=1):
        print("[Controls] Move left")
        self.set_velocity(0, -speed, 0, duration)

    def move_up(self, speed=1, duration=1):
        print("[Controls] Move up")
        self.set_velocity(0, 0, -speed, duration)

    def move_down(self, speed=1, duration=1):
        print("[Controls] Move down")
        self.set_velocity(0, 0, speed, duration)

    # -----------------------------
    # ORIENTATION CONTROL
    # -----------------------------
    def set_yaw(self, angle, relative=False):
        """
        Set heading/yaw.
        angle: degrees
        relative: True = relative yaw
        """
        is_relative = 1 if relative else 0
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle, 0, 1, is_relative,
            0, 0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # -----------------------------
    # TELEMETRY / STATE
    # -----------------------------
    def get_position(self):
        return self.vehicle.location.global_relative_frame

    def get_attitude(self):
        return self.vehicle.attitude

    def get_battery(self):
        return self.vehicle.battery

    def is_armed(self):
        return bool(self.vehicle.armed)

    def get_mode(self):
        return self.vehicle.mode.name

    # -----------------------------
    # SAFE UTILS
    # -----------------------------
    def wait_for_altitude(self, target_alt, timeout=30):
        start = time.time()
        while time.time() - start < timeout:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_alt:
                return True
            time.sleep(0.5)
        return False

    # -----------------------------
    # INTERNAL HELPERS
    # -----------------------------
    def _distance_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(max(0.0, 1-a)))
        return R * c
