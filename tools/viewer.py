"""Watch the rolling flydar cloud over UDP."""
import argparse
import socket
import struct
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore

HEADER = struct.Struct("<4sIQIH")
POINT = struct.Struct("<fffHB")


def decode(data):
    if len(data) < HEADER.size:
        return []
    magic, _, _, _, count = HEADER.unpack_from(data)
    if magic != b"FLY1" or not 1 <= count <= 12:
        return []
    if len(data) != HEADER.size + count * POINT.size:
        return []
    return [p[:3] for p in POINT.iter_unpack(data[HEADER.size:])]


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", type=int, default=4210)
    args = cli.parse_args()
    app = pg.mkQApp("flydar")
    window = gl.GLViewWidget()
    window.setWindowTitle("flydar / xyz")
    window.setCameraPosition(distance=5)
    dots = gl.GLScatterPlotItem(size=3)
    window.addItem(gl.GLGridItem())
    window.addItem(dots)
    window.show()
    points = deque(maxlen=23000)
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
                dots.setData(pos=np.asarray(points, dtype=np.float32))

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(32)
        app.exec()


if __name__ == "__main__":
    main()
