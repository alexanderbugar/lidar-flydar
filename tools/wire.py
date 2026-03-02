"""Common FLY1 packet and FDL1 capture layouts."""
import math
import struct
from dataclasses import dataclass

HEADER = struct.Struct("<4sIQIH")
POINT = struct.Struct("<fffHB")
MAX_PACKET = HEADER.size + 12 * POINT.size
LOG_MAGIC = b"FDL1"
RECORD = struct.Struct("<QI")


@dataclass(frozen=True)
class Packet:
    sequence: int
    start_us: int
    dropped: int
    points: tuple


def decode(data):
    if len(data) < HEADER.size:
        return None
    magic, sequence, start_us, dropped, count = HEADER.unpack_from(data)
    if magic != b"FLY1" or not 1 <= count <= 12:
        return None
    if len(data) != HEADER.size + count * POINT.size:
        return None
    points = tuple(POINT.iter_unpack(data[HEADER.size:]))
    if any(not all(map(math.isfinite, p[:3])) for p in points):
        return None
    return Packet(sequence, start_us, dropped, points)


def records(stream):
    if stream.read(len(LOG_MAGIC)) != LOG_MAGIC:
        raise ValueError("not an FDL1 log")
    last_us = 0
    while header := stream.read(RECORD.size):
        if len(header) != RECORD.size:
            raise ValueError("truncated record header")
        elapsed_us, size = RECORD.unpack(header)
        if not HEADER.size < size <= MAX_PACKET or elapsed_us < last_us:
            raise ValueError("bad record length or time")
        payload = stream.read(size)
        if len(payload) != size:
            raise ValueError("truncated packet")
        last_us = elapsed_us
        yield elapsed_us, payload
