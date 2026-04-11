"""Write one FDL1 capture to a binary little-endian PLY file."""
import argparse
import math
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


def downsample(points, size):
    import numpy as np

    data = np.asarray(points, dtype=np.float64)
    cells = np.floor(data[:, :3] / size).astype(np.int64)
    _, bucket, count = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    average = np.column_stack([
        np.bincount(bucket, weights=data[:, col]) / count
        for col in range(4)
    ])
    return [(x, y, z, round(strength)) for x, y, z, strength in average]


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
    cli.add_argument("--voxel", type=float, metavar="METRES")
    args = cli.parse_args()
    if args.voxel is not None and (not math.isfinite(args.voxel) or args.voxel <= 0):
        cli.error("voxel size must be finite and positive")
    try:
        with args.capture.open("rb") as stream:
            points = list(vertices(stream))
        if not points:
            raise ValueError("capture has no points")
        if args.voxel:
            points = downsample(points, args.voxel)
        with args.output.open("xb") as stream:
            write_ply(stream, points)
    except (OSError, ValueError) as exc:
        cli.error(str(exc))
    print(f"{len(points)} points")


if __name__ == "__main__":
    main()
