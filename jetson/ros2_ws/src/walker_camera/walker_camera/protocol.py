"""Wire format for the Jetson -> server JPEG stream.

Keep in sync with server/protocol.py.

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


def pack_frame(seq, sec, nanosec, jpeg):
    return HEADER.pack(MAGIC, seq & 0xFFFFFFFF, sec, nanosec, len(jpeg)) + jpeg
