"""Write one FDL1 capture to a binary little-endian PLY file."""
import argparse
import struct
from pathlib import Path

from wire import decode, records

VERTEX = struct.Struct("<fffB")


def vertices(stream):
    for _, data in records(stream):
        packet = decode(data)
        if packet is None:
            raise ValueError("bad FLY1 packet in capture")
        for x, y, z, _, intensity in packet.points:
            yield x, y, z, intensity


def write_ply(stream, points):
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar intensity\nend_header\n"
    )
    stream.write(header.encode("ascii"))
    for point in points:
        stream.write(VERTEX.pack(*point))


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("capture", type=Path)
    cli.add_argument("output", type=Path)
    args = cli.parse_args()
    try:
        with args.capture.open("rb") as stream:
            points = list(vertices(stream))
        if not points:
            raise ValueError("capture has no points")
        with args.output.open("xb") as stream:
            write_ply(stream, points)
    except (OSError, ValueError) as exc:
        cli.error(str(exc))
    print(f"{len(points)} points")


if __name__ == "__main__":
    main()
