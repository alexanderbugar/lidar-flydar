"""Watch the rolling flydar cloud over UDP."""
import argparse
import socket
import time
from contextlib import ExitStack
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore

from wire import LOG_MAGIC, RECORD, decode


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", type=int, default=4210)
    cli.add_argument("--record", metavar="FILE")
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
    with ExitStack() as stack:
        sock = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        sock.bind(("0.0.0.0", args.port))
        sock.setblocking(False)
        log = stack.enter_context(open(args.record, "xb")) if args.record else None
        if log:
            log.write(LOG_MAGIC)
        started = time.monotonic_ns()

        def update():
            for _ in range(256):
                try:
                    data = sock.recv(2048)
                except BlockingIOError:
                    break
                packet = decode(data)
                if packet is None:
                    continue
                if log:
                    elapsed_us = (time.monotonic_ns() - started) // 1000
                    log.write(RECORD.pack(elapsed_us, len(data)))
                    log.write(data)
                points.extend(p[:3] for p in packet.points)
            if points:
                dots.setData(pos=np.asarray(points, dtype=np.float32))

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(32)
        app.exec()


if __name__ == "__main__":
    main()
