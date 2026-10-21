"""A fake MG400 controller for ``--mock`` and the tests (loopback only, no hardware).

It listens on 127.0.0.1 only: a Dashboard port answering the read-only queries, and a
feedback port streaming 1440-byte packets every ``period_s``. There is deliberately no
motion port. Every received command is recorded, so tests can assert what was sent.
"""
import math
import socket
import threading
import time
from typing import List, Optional

from . import feedback

LOOPBACK = "127.0.0.1"


class FakeMG400:
    def __init__(self, mode: int = 4, pose=(308.141606, 0.384430, 67.673470, 18.632881),
                 angles=(0.071847, 10.278645, 16.846972, 18.555908, 0.0, 0.0),
                 errors: Optional[list] = None, period_s: float = 0.008,
                 split_replies: bool = False, echo_bare=(), single_dashboard: bool = False):
        self.mode = mode
        self.pose = list(pose)
        self.angles = list(angles)
        self.errors = errors if errors is not None else [[], [], [], [], [], []]
        self.period_s = period_s
        self.split_replies = split_replies
        self.echo_bare = set(echo_bare)            # names echoed without their parameters
        self.received: List[str] = []
        self.received_on: List[tuple] = []        # (Dashboard connection number from 1, cmd)
        # True: a new Dashboard connection closes the previous one (as the bench controller
        # appeared to do), then sets ``dropped``
        self.single_dashboard = single_dashboard
        self.dropped = threading.Event()
        self.conn_number = threading.local()       # .n: the connection whose command reply() answers
        self._dashboard_conns: List[socket.socket] = []
        self._silenced: set = set()
        self.feedback_bytes_received = 0
        self.hold_replies = threading.Event()      # set: replies wait until cleared
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._socks: List[socket.socket] = []
        self._feedback_conns: List[socket.socket] = []
        self.dashboard_port = self._listen(self._serve_dashboard)
        self.feedback_port = self._listen(self._serve_feedback)

    # -- lifecycle --------------------------------------------------------
    def _listen(self, handler) -> int:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((LOOPBACK, 0))
        srv.listen(8)
        srv.settimeout(0.1)
        self._socks.append(srv)
        t = threading.Thread(target=self._accept, args=(srv, handler), daemon=True)
        t.start()
        self._threads.append(t)
        return srv.getsockname()[1]

    def _accept(self, srv, handler):
        while not self._stop.is_set():
            try:
                conn, _ = srv.accept()
            except (socket.timeout, OSError):
                continue
            self._socks.append(conn)
            threading.Thread(target=handler, args=(conn,), daemon=True).start()

    def drop_feedback(self):
        """Close every feedback connection (the tool sees the feed end)"""
        for c in list(self._feedback_conns):
            try:
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            except OSError:
                pass

    def close(self):
        self._stop.set()
        for s in self._socks:
            try:
                s.close()
            except OSError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- Dashboard --------------------------------------------------------
    def _serve_dashboard(self, conn):
        with self._lock:
            older = list(self._dashboard_conns)
            self._dashboard_conns.append(conn)
            self.conn_number.n = len(self._dashboard_conns)
        number = self.conn_number.n
        if self.single_dashboard and older:
            for c in older:
                # end of stream for the client (what the bench run saw); whatever it still
                # sends is read and dropped, so it never sees a reset
                self._silenced.add(c)
                try:
                    c.shutdown(socket.SHUT_WR)
                except OSError:
                    pass
            self.dropped.set()
        conn.settimeout(0.1)
        buf = ""
        while not self._stop.is_set():
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            if conn in self._silenced:
                continue
            buf += data.decode("ascii", "replace")
            while ")" in buf:
                # a command ends at its only ")" (the allowed ones have no nested parentheses)
                cmd, buf = buf[:buf.index(")") + 1].strip(), buf[buf.index(")") + 1:]
                with self._lock:
                    self.received.append(cmd)
                    self.received_on.append((number, cmd))
                while self.hold_replies.is_set() and not self._stop.is_set():
                    time.sleep(0.01)
                reply = self.reply(cmd).encode("ascii")
                try:
                    if self.split_replies and len(reply) > 4:
                        conn.sendall(reply[:3])
                        time.sleep(0.005)
                        conn.sendall(reply[3:])
                    else:
                        conn.sendall(reply)
                except OSError:
                    return

    def reply(self, cmd: str) -> str:
        name = cmd.split("(", 1)[0]
        fmt = lambda xs: ",".join(f"{x:.6f}" for x in xs)
        if name == "RobotMode":
            return f"0,{{{self.mode}}},{cmd};"
        if name == "GetErrorID":
            inner = ",\n\t".join("[" + ",".join(str(i) for i in e) + "]" for e in self.errors)
            return f"0,{{[\n\t{inner}\n]\n}},GetErrorID();"
        if name == "GetPose":
            return f"0,{{{fmt(self.pose)}}},GetPose();"
        if name == "GetAngle":
            return f"0,{{{fmt(self.angles)}}},GetAngle();"
        if name == "PositiveSolution":
            j = [float(v) for v in cmd[cmd.index("(") + 1:-1].split(",")[:4]]
            return f"0,{{{fmt(self.fk(j))}}},{cmd};"
        if name == "InverseSolution":
            near = cmd[cmd.index("{") + 1:cmd.index("}")]
            return f"0,{{{near}}},{name + '()' if name in self.echo_bare else cmd};"
        return f"-10000,{{}},{cmd};"

    @staticmethod
    def fk(j):
        """Not the MG400's kinematics: a smooth stand-in so the tool has numbers to record"""
        r = 200.0 + 175.0 * math.sin(math.radians(j[1])) + 175.0 * math.cos(math.radians(j[2]))
        z = 100.0 + 175.0 * math.cos(math.radians(j[1])) - 175.0 * math.sin(math.radians(j[2]))
        a = math.radians(j[0])
        return [r * math.cos(a), r * math.sin(a), z, j[0] + j[3]]

    # -- feedback ---------------------------------------------------------
    def packet(self) -> bytes:
        return feedback.build(robot_mode=self.mode, timestamp_ms=int(time.time() * 1000),
                              q_actual=self.angles, tool_vector_actual=self.pose + [0.0, 0.0],
                              robot_type=1, error_status=0 if self.mode != 9 else 1)

    def _serve_feedback(self, conn):
        self._feedback_conns.append(conn)
        while not self._stop.is_set():
            try:
                conn.setblocking(False)
                data = conn.recv(4096)                   # the tool must never write here
                if data == b"":
                    return
                with self._lock:
                    self.feedback_bytes_received += len(data)
            except BlockingIOError:
                pass
            except OSError:
                return
            finally:
                try:
                    conn.setblocking(True)
                except OSError:
                    return
            try:
                pkt = self.packet()
                conn.sendall(pkt[:700])                  # split on purpose
                conn.sendall(pkt[700:])
            except OSError:
                return
            time.sleep(self.period_s)
