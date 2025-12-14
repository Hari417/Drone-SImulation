import socket
import threading
import sys
import json
import time

# =============== Helpers ===============

def send_msg(sock, data):
    """
    Prefix JSON with 4-byte length header
    and send over TCP
    """
    msg = json.dumps(data).encode("utf-8")
    length = len(msg).to_bytes(4, byteorder="big")
    sock.sendall(length + msg)

def recv_msg(sock):
    """
    Receive exactly one length-prefixed JSON message (4 byte header)
    """
    header = sock.recv(4)
    if not header:
        return None
    length = int.from_bytes(header, "big")
    body = b""
    while len(body) < length:
        chunk = sock.recv(length - len(body))
        if not chunk:
            return None
        body += chunk
    return json.loads(body.decode("utf-8"))

# =============== Agent ===============

class Agent:
    def __init__(self, my_id, my_port, peer_ports):
        self.my_id = my_id
        self.my_port = my_port
        self.peer_ports = peer_ports  # [(id, port), ...]

        self.out_socks = {}  # peer_id -> socket
        self.leader = None

    def listener(self):
        """
        Listen loop for incoming connections
        """
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", self.my_port))
        server.listen()
        print(f"{self.my_id}: listening on {self.my_port}")

        while True:
            sock, addr = server.accept()
            threading.Thread(target=self.handle_peer, args=(sock,), daemon=True).start()

    def handle_peer(self, sock):
        """
        Listen to one peer forever
        """
        while True:
            msg = recv_msg(sock)
            if not msg:
                break
            print(f"{self.my_id} received:", msg)

    def connect_peers(self):
        """
        Attempt outgoing connections to peers
        """
        for pid, pport in self.peer_ports:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect(("127.0.0.1", pport))
                self.out_socks[pid] = sock
                print(f"{self.my_id} connected -> {pid}:{pport}")
            except Exception as e:
                print(f"{self.my_id} failed to connect {pid}:{pport}")

    def elect_leader(self):
        """
        Lexicographically minimal ID as leader
        """
        all_ids = [self.my_id] + [p[0] for p in self.peer_ports]
        self.leader = sorted(all_ids)[0]
        print(f"{self.my_id} elected leader: {self.leader}")

    def announce_leader(self):
        """
        Send leader message to peers
        """
        msg = {"type": "leader_announce", "leader": self.leader}
        for pid, sock in self.out_socks.items():
            send_msg(sock, msg)

    def run(self):
        # Start listening thread
        threading.Thread(target=self.listener, daemon=True).start()
        time.sleep(0.5)

        # Connect to known peers
        self.connect_peers()
        time.sleep(0.5)

        # Elect and announce
        self.elect_leader()
        self.announce_leader()

        # Keep alive to receive
        time.sleep(5)
        print(f"{self.my_id} done.")

# =============== Entrypoint ===============

if __name__ == "__main__":
    """
    Usage:
      python agent.py <id> <my_port> <peer1_id>:<peer1_port> <peer2_id>:<peer2_port> ...
    """
    my_id = sys.argv[1]
    my_port = int(sys.argv[2])
    peer_specs = sys.argv[3:]
    peers = []
    for spec in peer_specs:
        pid, pport = spec.split(":")
        peers.append((pid, int(pport)))

    agent = Agent(my_id, my_port, peers)
    agent.run()
