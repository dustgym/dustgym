"""Transport for CCSDS Space Packets between the ground station and the rover.

Two implementations behind one ``Link`` interface:

* ``LoopbackLink`` — in-process, thread-safe. It packs every packet to real octets and unpacks on
  receive, so the CCSDS codec is genuinely exercised even in the single-process demo and the tests.
  Deterministic (no wall-clock delay).
* ``UdpLink`` — one Space Packet per UDP datagram, with a configurable one-way light-time delay
  (Earth–Moon is ~1.3 s one way) and optional drop rate, for the containerized run.

``send`` takes a ``ccsds.SpacePacket``; ``recv`` returns one (or ``None`` on timeout).
"""
from __future__ import annotations

import heapq
import socket
import threading
import time
from collections import deque

import ccsds


class Link:
    """Bidirectional Space Packet transport interface."""

    def send(self, pkt: ccsds.SpacePacket) -> None:
        raise NotImplementedError

    def recv(self, timeout: float | None = None) -> "ccsds.SpacePacket | None":
        raise NotImplementedError

    def close(self) -> None:
        pass


class _Pipe:
    """A thread-safe byte queue with blocking, timeout-able receive."""

    def __init__(self) -> None:
        self._q: deque[bytes] = deque()
        self._cv = threading.Condition()

    def put(self, raw: bytes) -> None:
        with self._cv:
            self._q.append(raw)
            self._cv.notify()

    def get(self, timeout: float | None) -> "bytes | None":
        with self._cv:
            if not self._q:
                self._cv.wait(timeout)
            return self._q.popleft() if self._q else None


class LoopbackLink(Link):
    """One end of an in-process pair. Create both ends with :func:`loopback_pair`."""

    def __init__(self, tx: _Pipe, rx: _Pipe) -> None:
        self._tx = tx
        self._rx = rx

    def send(self, pkt: ccsds.SpacePacket) -> None:
        self._tx.put(pkt.pack())                 # real wire octets

    def recv(self, timeout: float | None = None) -> "ccsds.SpacePacket | None":
        raw = self._rx.get(timeout)
        return ccsds.SpacePacket.unpack(raw) if raw is not None else None


def loopback_pair() -> tuple[LoopbackLink, LoopbackLink]:
    """Return (ground_link, flight_link) wired so each one's send is the other's recv."""
    g2f, f2g = _Pipe(), _Pipe()
    ground = LoopbackLink(tx=g2f, rx=f2g)
    flight = LoopbackLink(tx=f2g, rx=g2f)
    return ground, flight


class UdpLink(Link):
    """Datagram transport with a simulated one-way light-time delay (move-and-wait realism)."""

    def __init__(self, local_addr: tuple[str, int], remote_addr: tuple[str, int], *,
                 light_time_s: float = 0.0) -> None:
        self.remote_addr = remote_addr
        self.light_time_s = float(light_time_s)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(local_addr)
        self._delayed: list[tuple[float, int, bytes]] = []   # heap of (deliver_at, seq, raw)
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sender = threading.Thread(target=self._drain_loop, daemon=True)
        self._sender.start()

    def _drain_loop(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            due: list[bytes] = []
            with self._lock:
                while self._delayed and self._delayed[0][0] <= now:
                    due.append(heapq.heappop(self._delayed)[2])
            for raw in due:
                try:
                    self._sock.sendto(raw, self.remote_addr)
                except OSError:
                    pass
            time.sleep(0.005)

    def send(self, pkt: ccsds.SpacePacket) -> None:
        raw = pkt.pack()
        if self.light_time_s <= 0.0:
            self._sock.sendto(raw, self.remote_addr)
            return
        with self._lock:
            heapq.heappush(self._delayed, (time.monotonic() + self.light_time_s, self._seq, raw))
            self._seq += 1

    def recv(self, timeout: float | None = None) -> "ccsds.SpacePacket | None":
        # Best-effort datagram poll. timeout==0.0 -> non-blocking (empty raises BlockingIOError, not
        # socket.timeout); a closed socket during shutdown raises OSError(EBADF). OSError covers all of
        # these (TimeoutError/socket.timeout/BlockingIOError are subclasses) -> return None on any of them.
        if self._stop.is_set():
            return None
        try:
            self._sock.settimeout(timeout)            # settimeout itself raises EBADF on a closed socket
            raw, _ = self._sock.recvfrom(65535)
        except OSError:
            return None
        return ccsds.SpacePacket.unpack(raw)

    def close(self) -> None:
        self._stop.set()
        self._sock.close()
