"""Wire format for the Jetson -> server JPEG stream.

Keep in sync with jetson/ros2_ws/src/walker_camera/walker_camera/protocol.py.

Each frame is a fixed header followed by `length` bytes of JPEG:
    magic    4s   b'WLKR'
    seq      uint32
    sec      int32   header.stamp.sec of the source image
    nanosec  uint32  header.stamp.nanosec
    length   uint32
All fields are network byte order.
"""
import struct

MAGIC = b'WLKR'
HEADER = struct.Struct('!4sIiII')
MAX_FRAME_BYTES = 32 * 1024 * 1024


def pack_frame(seq, sec, nanosec, jpeg):
    return HEADER.pack(MAGIC, seq & 0xFFFFFFFF, sec, nanosec, len(jpeg)) + jpeg


def recv_exact(sock, n):
    """Read exactly n bytes, or return None if the peer closed the connection."""
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        r = sock.recv_into(view[got:], n - got)
        if r == 0:
            return None
        got += r
    return bytes(buf)


def read_frame(sock):
    """Return (seq, sec, nanosec, jpeg) or None on EOF. Raises ValueError on a bad header."""
    raw = recv_exact(sock, HEADER.size)
    if raw is None:
        return None
    magic, seq, sec, nanosec, length = HEADER.unpack(raw)
    if magic != MAGIC:
        raise ValueError(f'bad magic {magic!r}')
    if length > MAX_FRAME_BYTES:
        raise ValueError(f'frame too large ({length} bytes)')
    jpeg = recv_exact(sock, length)
    if jpeg is None:
        return None
    return seq, sec, nanosec, jpeg
