#!/usr/bin/env python3

import argparse
import yaml
import json
import socket
import threading
import time
from multi_coverage import MultiDronePlanner
from Controls import DroneController

# -----------------------------------
# JSON Length-Prefixed Messaging Utils
# -----------------------------------

def send_message(sock, data):
    """
    Serialize dict to JSON, prefix with length, send safe over TCP.
    """
    try:
        payload = json.dumps(data).encode("utf-8")
        length = len(payload).to_bytes(4, "big")
        sock.sendall(length + payload)
    except (BrokenPipeError, OSError):
        # peer likely disconnected
        pass

def receive_message(sock):
    """
    Read exactly one length-prefixed JSON message from the socket.
    """
    try:
        header = sock.recv(4)
        if not header or len(header) < 4:
            return None
        length = int.from_bytes(header, "big")

        data = b""
        while len(data) < length:
            chunk = sock.recv(length - len(data))
            if not chunk:
                return None
            data += chunk

        return json.loads(data.decode("utf-8"))
    except (ConnectionResetError, OSError, json.JSONDecodeError):
        return None

# -----------------------------------
# TCP Listener Thread
# -----------------------------------

class ListenerThread(threading.Thread):
    """
    Thread that listens on a TCP port and dispatches connections.
    """
    def __init__(self, host, port, handler):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.handler = handler
        self._stop_event = threading.Event()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allow immediate reuse of the port on shutdown
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen()
        print(f"[{host}:{port}] Listening for swarm connections")

    def run(self):
        while not self._stop_event.is_set():
            try:
                self.server.settimeout(0.5)
                client_sock, _ = self.server.accept()
                client_sock.settimeout(None)
                threading.Thread(
                    target=self.handler,
                    args=(client_sock,),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except OSError:
                break

    def stop(self):
        self._stop_event.set()
        try:
            self.server.close()
        except OSError:
            pass

# -----------------------------------
# Drone Agent
# -----------------------------------

class DroneAgent:
    def __init__(self, fleet, mission_cfg, my_id):
        self.all_drones = fleet
        self.mission_cfg = mission_cfg
        self.my_id = my_id

        # Extract own config
        self.my_conf = next(d for d in self.all_drones if d["drone_id"] == my_id)
        self.peers = [d for d in self.all_drones if d["drone_id"] != my_id]

        self.listener = None
        self.running = True
        self.peer_sockets = {}
        self.assignment = None
        self.leader_id = None

    def start_listener(self):
        """
        Start listening for incoming TCP connections.
        """
        self.listener = ListenerThread(
            self.my_conf["coord_ip"],
            self.my_conf["coord_port"],
            self.handle_peer_socket
        )
        self.listener.start()

    def handle_peer_socket(self, sock):
        """
        Handle messages from one peer socket.
        """
        while self.running:
            msg = receive_message(sock)
            if msg is None:
                break
            self.process_message(msg, sock)
        try:
            sock.close()
        except OSError:
            pass

    def connect_peers(self):
        """
        Make outgoing connections to all peers.
        """
        for peer in self.peers:
            peer_id = peer["drone_id"]
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                s.connect((peer["coord_ip"], peer["coord_port"]))
                s.settimeout(None)  # back to blocking
                self.peer_sockets[peer_id] = s
                print(f"[{self.my_id}] Connected to {peer_id}")
            except OSError:
                pass

    def process_message(self, msg, sock):
        msg_type = msg.get("type")
        if msg_type == "leader_announcement":
            self.leader_id = msg["leader_id"]
        elif msg_type == "assignment":
            if msg.get("drone_id") == self.my_id:
                self.assignment = msg.get("waypoints")

    def elect_leader(self):
        """
        Determine leader by lexicographically smallest ID.
        """
        all_ids = [d["drone_id"] for d in self.all_drones]
        self.leader_id = sorted(all_ids)[0]

    def announce_leader(self):
        announcement = {"type": "leader_announcement", "leader_id": self.leader_id}
        for sock in self.peer_sockets.values():
            send_message(sock, announcement)

    def assign_waypoints(self):
        planner = MultiDronePlanner(
            geofence=(
                self.mission_cfg["geofence"]["lat_min"],
                self.mission_cfg["geofence"]["lon_min"],
                self.mission_cfg["geofence"]["lat_max"],
                self.mission_cfg["geofence"]["lon_max"],
            ),
            num_drones=len(self.all_drones),
            altitude=self.mission_cfg["altitude"],
            fov_deg=self.mission_cfg["camera_fov"],
            overlap=self.mission_cfg["overlap"]
        )
        plans = planner.allocate()
        assignments = {
            d["drone_id"]: plans[idx]["waypoints"]
            for idx, d in enumerate(self.all_drones)
        }

        for peer_id, sock in self.peer_sockets.items():
            send_message(sock, {
                "type": "assignment",
                "drone_id": peer_id,
                "waypoints": assignments[peer_id]
            })

        # leader does its own too
        self.assignment = assignments[self.leader_id]

    def execute_assignment(self):
        if not self.assignment:
            print(f"[{self.my_id}] No assignment to run")
            return

        drone = DroneController(self.my_conf["mavlink"], default_alt=self.mission_cfg["altitude"])
        drone.connect()
        drone.takeoff(self.mission_cfg["altitude"])
        for lat, lon in self.assignment:
            print(f"[{self.my_id}] Moving to {lat}, {lon}")
            drone.goto(lat, lon)
        drone.land()
        drone.disconnect()

    def shutdown(self):
        """
        Stop listener and close sockets cleanly.
        """
        self.running = False
        if self.listener:
            self.listener.stop()
        for s in self.peer_sockets.values():
            try:
                s.shutdown(socket.SHUT_RDWR)
                s.close()
            except OSError:
                pass

    def run(self):
        try:
            self.start_listener()
            time.sleep(1)
            self.connect_peers()
            self.elect_leader()

            # Leader sends assignments
            if self.leader_id == self.my_id:
                self.announce_leader()
                self.assign_waypoints()
                self.execute_assignment()
            else:
                timeout = time.time() + 10
                while self.assignment is None and time.time() < timeout:
                    time.sleep(0.2)
                self.execute_assignment()

        finally:
            # Always clean up
            self.shutdown()

# -----------------------------------
# ENTRYPOINT
# -----------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Fleet config YAML")
    parser.add_argument("--id", required=True, help="Drone ID")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        conf = yaml.safe_load(f)

    drone_list = conf["drones"]
    mission_conf = conf["mission"]

    agent = DroneAgent(drone_list, mission_conf, args.id)
    agent.run()
