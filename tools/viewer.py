"""Watch a rolling cloud. Pass --record to keep the UDP datagrams."""
import argparse
import socket
import time
from collections import deque
from contextlib import ExitStack

import numpy as np

from wire import LOG_MAGIC, RECORD, decode


def receive(sock, points, log, started):
    changed = False
    for _ in range(256):
        try:
            data, _ = sock.recvfrom(2048)
        except BlockingIOError:
            break
        packet = decode(data)
        if packet is None:
            continue
        if log:
            elapsed = (time.monotonic_ns() - started) // 1000
            log.write(RECORD.pack(elapsed, len(data)))
            log.write(data)
        points.extend((x, y, z, intensity) for x, y, z, _, intensity in packet.points)
        changed = True
    return changed


def watch(args):
    import open3d as o3d

    points = deque(maxlen=args.points)
    with ExitStack() as stack:
        sock = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        sock.bind(("0.0.0.0", args.port))
        sock.setblocking(False)

        window = o3d.visualization.Visualizer()
        if not window.create_window(window_name="flydar", width=1050, height=720):
            raise RuntimeError("open3d couldn't open a window")
        stack.callback(window.destroy_window)
        options = window.get_render_option()
        options.background_color = np.array([0.06, 0.07, 0.08])
        options.point_size = 2.0

        log = stack.enter_context(open(args.record, "xb")) if args.record else None
        if log:
            log.write(LOG_MAGIC)
        started = time.monotonic_ns()

        cloud = o3d.geometry.PointCloud()
        added = dirty = False
        last_draw = time.monotonic()
        while window.poll_events():
            dirty |= receive(sock, points, log, started)
            now = time.monotonic()
            if dirty and now - last_draw >= 1/30:
                data = np.asarray(points, dtype=np.float64)
                cloud.points = o3d.utility.Vector3dVector(data[:, :3].copy())
                strength = 0.25 + 0.75*data[:, 3] / 255
                colors = strength[:, None] * [0.65, 0.92, 1.0]
                cloud.colors = o3d.utility.Vector3dVector(colors)
                if added:
                    window.update_geometry(cloud)
                else:
                    added = window.add_geometry(cloud)
                    window.get_view_control().set_up([0, 0, 1])
                last_draw = now
                dirty = False
            window.update_renderer()
            time.sleep(0.004)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", type=int, default=4210)
    cli.add_argument("--points", type=int, default=24000)
    cli.add_argument("--record", metavar="FILE")
    args = cli.parse_args()
    if args.points < 1:
        cli.error("points must be positive")
    try:
        watch(args)
    except KeyboardInterrupt:
        return
    except (OSError, RuntimeError) as exc:
        cli.error(str(exc))


if __name__ == "__main__":
    main()
