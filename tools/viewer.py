"""Watch the live XY scan over UDP."""
import argparse
import math
import socket
import struct
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

HEADER = struct.Struct("<IH")
POINT = struct.Struct("<HHB")
COUNT = 12


def decode(data):
    if len(data) != HEADER.size + COUNT * POINT.size:
        return []
    points = []
    for angle, mm, _ in POINT.iter_unpack(data[HEADER.size:]):
        if mm:
            rad = math.radians(angle / 100)
            points.append((mm * math.cos(rad), mm * math.sin(rad)))
    return points


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", type=int, default=4210)
    args = cli.parse_args()
    app = pg.mkQApp("flydar")
    window = pg.plot(title="flydar / xy")
    window.setAspectLocked()
    dots = window.plot(pen=None, symbol="o", symbolSize=3)
    points = deque(maxlen=2800)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", args.port))
        sock.setblocking(False)

        def update():
            for _ in range(256):
                try:
                    data = sock.recv(2048)
                except BlockingIOError:
                    break
                points.extend(decode(data))
            if points:
                x, y = zip(*points)
                dots.setData(x, y)

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(32)
        app.exec()


if __name__ == "__main__":
    main()
